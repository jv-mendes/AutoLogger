// mpu_madgwick.c
#include "MPU6050.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stm32g4xx_hal.h" // ou seu HAL header
#include "mpu_madgwick.h"

// ======================================================================
// Configurações
// ======================================================================
#define SAMPLE_HZ      100.0f    // frequência alvo (Hz)
#define SAMPLE_DT      (1.0f/SAMPLE_HZ)
#define MADGWICK_BETA  0.08f     // Ganho Beta (0.04..0.2 típico). Ajuste conforme necessidade.
#define G               9.80665f

// ======================================================================
// Estados / variáveis globais
// ======================================================================
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion (est. orientação)
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f; // bias em °/s

extern I2C_HandleTypeDef hi2c1; // garante que seu hi2c1 é visível

// Leitura atual (convertida)
float Ax_g, Ay_g, Az_g;   // acelerômetro em g (unitário)
float Gx_dps, Gy_dps, Gz_dps; // giroscópio em °/s

// Saídas
float linAx_ms2 = 0.0f, linAy_ms2 = 0.0f, linAz_ms2 = 0.0f;
float pitch_deg = 0.0f, roll_deg = 0.0f, yaw_deg = 0.0f;

// ======================================================================
// Madgwick AHRS (IMU-only) - atualização do quaternion
// Inputs: gx,gy,gz (rad/s) ; ax,ay,az (g, normalized approx)
// dt = intervalo em segundos
// ======================================================================
void MadgwickAHRSupdateIMU(float gx, float gy, float gz,
                           float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // Rate of change from gyroscope
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Normalise accelerometer measurement
    recipNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (recipNorm == 0.0f) return; // evita divisão por zero
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Gradient decent algorithm corrective step
    float _2q0q2 = 2.0f * q0 * q2;
    float _2q2q3 = 2.0f * q2 * q3;
    float f1 = _2q1 * q3 - _2q0 * q2 - ax;
    float f2 = _2q0 * q1 + _2q2 * q3 - ay;
    float f3 = 1.0f - _2q1 * q1 - _2q2 * q2 - az;

    // compute jacobian * f (approx)
    s0 = -_2q2 * f1 + _2q1 * f2;
    s1 =  _2q3 * f1 + _2q0 * f2 - _4q1 * f3;
    s2 = -_2q0 * f1 + _2q3 * f2 - _4q2 * f3;
    s3 =  _2q1 * f1 + _2q2 * f2;

    // normalize step magnitude
    recipNorm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (recipNorm != 0.0f) {
        recipNorm = 1.0f / recipNorm;
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
    }

    // Apply feedback step
    qDot0 -= MADGWICK_BETA * s0;
    qDot1 -= MADGWICK_BETA * s1;
    qDot2 -= MADGWICK_BETA * s2;
    qDot3 -= MADGWICK_BETA * s3;

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normalise quaternion
    recipNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (recipNorm == 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// ======================================================================
// Quaternion -> Euler angles (deg). Returns roll, pitch, yaw
// ======================================================================
void QuaternionToEulerDegrees(float *roll, float *pitch, float *yaw)
{
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG(1.0f);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1)
        *pitch = copysignf(90.0f, sinp); // use 90 degrees if out of range
    else
        *pitch = asinf(sinp) * RAD2DEG(1.0f);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG(1.0f);
}

// ======================================================================
// Calibra giroscópio (bias) simples: média de N amostras com sensor parado
// Retorna valores em deg/s (subtrair de leituras brutas depois)
// ======================================================================
void CalibrateGyro(int samples)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    for (int i = 0; i < samples; i++) {
        MPU6050_Read_Gyro(&Gx_dps, &Gy_dps, &Gz_dps);
        sx += Gx_dps;
        sy += Gy_dps;
        sz += Gz_dps;
        HAL_Delay(5);
    }
    gyroBiasX = sx / (float)samples;
    gyroBiasY = sy / (float)samples;
    gyroBiasZ = sz / (float)samples;
}

// ======================================================================
// Loop principal de atualização: chama leitura, filtro, subtrai gravidade
// Preenchendo: linAx_ms2, linAy_ms2, linAz_ms2, pitch_deg, roll_deg, yaw_deg
// ======================================================================
void IMU_Update(float dt)
{
    // 1) Leitura (Ax/Ay/Az em g; Gx/Gy/Gz em °/s)
    MPU6050_Read_Accel(&Ax_g, &Ay_g, &Az_g);
    MPU6050_Read_Gyro(&Gx_dps, &Gy_dps, &Gz_dps);

    // 2) Remover bias do giroscópio e converter para rad/s
    float gx = (Gx_dps - gyroBiasX) * (M_PI / 180.0f);
    float gy = (Gy_dps - gyroBiasY) * (M_PI / 180.0f);
    float gz = (Gz_dps - gyroBiasZ) * (M_PI / 180.0f);

    // 3) Atualiza Madgwick (ax,ay,az em g)
    MadgwickAHRSupdateIMU(gx, gy, gz, Ax_g, Ay_g, Az_g, dt);

    // 4) Extrai Euler (graus)
    QuaternionToEulerDegrees(&roll_deg, &pitch_deg, &yaw_deg);

    // 5) Estima vetor da gravidade no frame do sensor a partir do quaternion
    // vetor gravidade (g) no corpo = [2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), q0^2 - q1^2 - q2^2 + q3^2]
    float gX = 2.0f * (q1 * q3 - q0 * q2);
    float gY = 2.0f * (q0 * q1 + q2 * q3);
    float gZ = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 6) Aceleração linear (g) = medida (g) - gravidade (g)
    float linax_g = Ax_g - gX;
    float linay_g = Ay_g - gY;
    float linaz_g = Az_g - gZ;

    // 7) Converter para m/s²
    linAx_ms2 = linax_g * G;
    linAy_ms2 = linay_g * G;
    linAz_ms2 = linaz_g * G;
}
