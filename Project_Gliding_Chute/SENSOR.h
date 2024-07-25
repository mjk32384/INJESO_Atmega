/*
 * SENSOR.h
 *
 * Created: 2024-07-20 오전 1:46:40
 *  Author: stu11
 */ 
// SENSOR.h

#ifndef SENSOR_H
#define SENSOR_H

#include <avr/io.h>
#include <util/delay.h>

// I2C addresses
#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS  0x0C
#define GYRO_CALIBRATION_SAMPLES 516


// MPU9250 Functions
void MPU9250_Write(uint8_t reg, uint8_t data);
uint8_t MPU9250_Read(uint8_t reg);
void MPU9250_Init();
void Read_Accel_Gyro(int16_t* accel, int16_t* gyro);
void Calibrate_Gyro(int16_t* gyroBias);

// AK8963 Functions
void AK8963_Write(uint8_t reg, uint8_t data);
uint8_t AK8963_Read(uint8_t reg);
void AK8963_Init();
void Read_Magnetometer(int16_t* mag);
void Calibrate_AK8963(float* magBias, float* magScale);
#endif // SENSOR_H
