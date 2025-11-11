#ifndef MPU_MADGWICK_H_
#define MPU_MADGWICK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "MPU6050.h"
#include "math.h"
#include "stdint.h"

// ================================================================
// Configurações principais
// ================================================================
#define G               9.80665f        // aceleração da gravidade (m/s²)
#define SAMPLE_HZ       100.0f          // frequência de atualização (Hz)
#define SAMPLE_DT       (1.0f / SAMPLE_HZ)
#define MADGWICK_BETA   0.08f           // ganho do filtro Madgwick (ajustável)
#define RAD2DEG(x)      ((x) * 57.295779513f)
#define DEG2RAD(x)      ((x) * 0.01745329252f)

// ================================================================
// Variáveis globais (para leitura externa)
// ================================================================

// Leituras filtradas do sensor
extern float Ax_g, Ay_g, Az_g;           // Acelerômetro (g)
extern float Gx_dps, Gy_dps, Gz_dps;     // Giroscópio (°/s)

// Resultados do filtro (saídas principais)
extern float pitch_deg, roll_deg, yaw_deg;        // Orientação (°)
extern float linAx_ms2, linAy_ms2, linAz_ms2;     // Aceleração linear (m/s²)

// Bias do giroscópio (em °/s)
extern float gyroBiasX, gyroBiasY, gyroBiasZ;

// ================================================================
// Funções públicas
// ================================================================

/**
 * @brief Inicializa o filtro e variáveis (opcional, você pode usar apenas MPU6050_init)
 */
void IMU_Init(void);

/**
 * @brief Calibra o giroscópio (média de N amostras com o sensor parado)
 * @param samples número de amostras para média (recomendado: 200)
 */
void CalibrateGyro(int samples);

/**
 * @brief Atualiza o filtro Madgwick e calcula aceleração linear + ângulos
 * @param dt Intervalo de tempo desde a última atualização (em segundos)
 */
void IMU_Update(float dt);

/**
 * @brief Atualiza o filtro Madgwick (modo interno) usando acel+giro
 * @param gx,gy,gz Giroscópio (rad/s)
 * @param ax,ay,az Acelerômetro (g)
 * @param dt Intervalo de tempo em segundos
 */
void MadgwickAHRSupdateIMU(float gx, float gy, float gz,
                           float ax, float ay, float az, float dt);

/**
 * @brief Converte o quaternion atual para ângulos de Euler (graus)
 * @param roll ponteiro para ângulo de rotação em X
 * @param pitch ponteiro para ângulo de rotação em Y
 * @param yaw ponteiro para ângulo de rotação em Z
 */
void QuaternionToEulerDegrees(float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* MPU_MADGWICK_H_ */
