#include "MPU6050.h"
#define MPU6050_ADDR (0x68 << 1)

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define DEG2RAD(x) ((x) * 0.01745329252f)
#define RAD2DEG(x) ((x) * 57.295779513f)

//
#define SAMPLE_HZ     100   // aquisition frequency
#define AVG_PERIOD_HZ 1     // calculate mean at 1hz
#define MAX_SAMPLES   (SAMPLE_HZ / AVG_PERIOD_HZ)


int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

// Buffers
static float pitch_buf[MAX_SAMPLES];
static float roll_buf[MAX_SAMPLES];
static float yaw_buf[MAX_SAMPLES];
static float ax_buf[MAX_SAMPLES];
static float ay_buf[MAX_SAMPLES];
static float az_buf[MAX_SAMPLES];

static int sample_index = 0;
static int sample_count = 0;

//sum for means
static float sum_pitch, sum_roll, sum_yaw, sum_ax, sum_ay, sum_az = 0.0f;

typedef struct {
    float refX, refY, refZ;  // vetor referência normalizado
} AccelReference;


extern float Ax, Ay, Az, Gx, Gy, Gz;
extern float relX, relY, relZ;
extern float accLinX, accLinY, accLinZ;
AccelReference accelRef = {0,0,1}; // padrão: Z para cima
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

extern float pitch = 0.0f, roll = 0.0f;  // Ângulos estimados
extern float alpha = 0.98f;              // Constante do filtro complementar
extern float dt = 0.01f;                 // Intervalo de tempo (10 ms)
extern float mean_pitch, mean_roll, mean_yaw, mean_ax, mean_ay, mean_az;

uint32_t last_time = 0;

void MPU6050_init(void)
{
	uint8_t check,data;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1 , 1000);
	if (check == 104)
	{
		//Power management register write all 0's to wake up sensor
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
		//Set data rate of 1KHz by writing SMPRT_DIV register
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
		//Writing both register with 0 to set full scale range
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
		
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}
     memset(pitch_buf, 0, sizeof(pitch_buf));
     memset(roll_buf,  0, sizeof(roll_buf));
     memset(yaw_buf,   0, sizeof(yaw_buf));
     memset(ax_buf,    0, sizeof(ax_buf));
     memset(ay_buf,    0, sizeof(ay_buf));
     memset(az_buf,    0, sizeof(az_buf));
}

//Function with multiple return using pointer

void MPU6050_Read_Accel (float* Ax, float* Ay, float* Az)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	//Adding 2 BYTES into 16 bit integer 
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
	
	//conversão para g
	*Ax = Accel_X_RAW/16384.0;
	*Ay = Accel_Y_RAW/16384.0;
	*Az = Accel_Z_RAW/16384.0;
}

void MPU6050_Read_Gyro(float* Gx, float* Gy, float* Gz)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    // Correctly assign raw data values for each axis
    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    //conversão para °/s
    *Gx = Gyro_X_RAW / 131.0;
    *Gy = Gyro_Y_RAW / 131.0;
    *Gz = Gyro_Z_RAW / 131.0;
}


void MPU6050_SetReference(float Ax, float Ay, float Az) {
    float mag = sqrtf(Ax*Ax + Ay*Ay + Az*Az);
    if (mag < 1e-6f) return; // proteção
    accelRef.refX = Ax / mag;
    accelRef.refY = Ay / mag;
    accelRef.refZ = Az / mag;
}

void MPU6050_Get_Accel_Relative(float Ax, float Ay, float Az,
                                float *relX, float *relY, float *relZ)
{
    // Normaliza vetor atual
    float mag = sqrtf(Ax*Ax + Ay*Ay + Az*Az);
    if (mag < 1e-6f) return;
    Ax /= mag;
    Ay /= mag;
    Az /= mag;

    // Produto vetorial (eixo de rotação)
    float rx = accelRef.refY*Az - accelRef.refZ*Ay;
    float ry = accelRef.refZ*Ax - accelRef.refX*Az;
    float rz = accelRef.refX*Ay - accelRef.refY*Ax;

    // Produto escalar (ângulo entre vetores)
    float dot = accelRef.refX*Ax + accelRef.refY*Ay + accelRef.refZ*Az;

    // “Erro” relativo (diferença vetorial)
    *relX = rx;  // componente ortogonal (rotação lateral)
    *relY = ry;
    *relZ = dot - 1.0f; // diferença em relação ao eixo Z (quanto está inclinado)
}

// Loop principal
void MPU6050_Get_Lin_Accel(float *accLinX, float *accLinY, float *accLinZ ) {
//void MPU6050_Get_Lin_Accel(void) {
    uint32_t now = HAL_GetTick();
    dt = (now - last_time) / 1000.0f;
    last_time = now;

    // Leitura dos sensores
    MPU6050_Read_Accel(&Ax, &Ay, &Az);
    MPU6050_Read_Gyro(&Gx, &Gy, &Gz);

    // Conversão giroscópio: °/s → rad/s
    float gx = DEG2RAD(Gx);
    float gy = DEG2RAD(Gy);
    float gz = DEG2RAD(Gz);

    // Cálculo dos ângulos a partir do acelerômetro
    float pitch_acc = atan2f(Ay, sqrtf(Ax*Ax + Az*Az));
    float roll_acc  = atan2f(-Ax, Az);

    // Filtro complementar
    pitch = alpha * (pitch + gx * dt) + (1.0f - alpha) * pitch_acc;
    roll  = alpha * (roll  + gy * dt) + (1.0f - alpha) * roll_acc;

    // --- Remoção da gravidade ---

    // Gravidade no frame do sensor (em g)
    float gX = -sinf(roll);
    float gY =  sinf(pitch);
    float gZ =  cosf(pitch) * cosf(roll);

    // Remove gravidade (mantém apenas aceleração linear)
    *accLinX = Ax - gX;
    *accLinY = Ay - gY;
    *accLinZ = Az - gZ;

    // Converte para m/s²
    *accLinX *= 9.80665f;
    *accLinY *= 9.80665f;
    *accLinZ *= 9.80665f;
}


/**
 * @brief Armazena nova leitura do IMU
 */
void MPU6050AddSample(float pitch_deg, float roll_deg, float yaw_deg,
                   float linAx_ms2, float linAy_ms2, float linAz_ms2)
{
    // Se o buffer estiver cheio, subtrai o valor antigo do somatório
    if (sample_count == MAX_SAMPLES) {
        sum_pitch -= pitch_buf[sample_index];
        sum_roll  -= roll_buf[sample_index];
        sum_yaw   -= yaw_buf[sample_index];
        sum_ax    -= ax_buf[sample_index];
        sum_ay    -= ay_buf[sample_index];
        sum_az    -= az_buf[sample_index];
    } else {
        sample_count++;
    }

    // Adiciona novo valor aos buffers
    pitch_buf[sample_index] = pitch_deg;
    roll_buf[sample_index]  = roll_deg;
    yaw_buf[sample_index]   = yaw_deg;
    ax_buf[sample_index]    = linAx_ms2;
    ay_buf[sample_index]    = linAy_ms2;
    az_buf[sample_index]    = linAz_ms2;

    // Atualiza os somatórios
    sum_pitch += pitch_deg;
    sum_roll  += roll_deg;
    sum_yaw   += yaw_deg;
    sum_ax    += linAx_ms2;
    sum_ay    += linAy_ms2;
    sum_az    += linAz_ms2;

    // Atualiza índice circular
    sample_index++;
    if (sample_index >= MAX_SAMPLES)
        sample_index = 0;
}

/**
 * @brief Calcula e imprime a média das últimas N amostras (1s)
 */
void MPU6050ComputeMean(float * mean_pitch, float * mean_roll, float * mean_yaw,
						float * mean_ax, float * mean_ay, float * mean_az)
{
    if (sample_count == 0) return; // Proteção

    *mean_pitch = sum_pitch / sample_count;
    *mean_roll  = sum_roll  / sample_count;
    *mean_yaw   = sum_yaw   / sample_count;
    *mean_ax    = sum_ax    / sample_count;
    *mean_ay    = sum_ay    / sample_count;
    *mean_az    = sum_az    / sample_count;
}
