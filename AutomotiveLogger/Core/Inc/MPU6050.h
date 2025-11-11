#include "stm32g4xx_hal.h"

void MPU6050_init(void); //Initialize the MPU 
void MPU6050_Read_Accel (float *Ax, float *Ay, float *Az); //Read MPU Accelerator 
void MPU6050_Read_Gyro (float *Gx, float *Gy, float *Gz); //Read MPU Gyroscope
void MPU6050_SetReference(float Ax, float Ay, float Az); //Set the reference for relative coordinate system
//Calculate the new Accel based os relative coordinate system
void MPU6050_Get_Accel_Relative(float Ax, float Ay, float Az,
                                float *relX, float *relY, float *relZ);
