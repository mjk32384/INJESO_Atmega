/*
 * SENSOR.c
 *
 * Created: 2024-07-20 오전 1:46:59
 *  Author: stu11
 */ 
// SENSOR.c
// SENSOR.c

#include "sensor.h"
#include "serial.h"

// MPU9250 Functions
void MPU9250_Write(uint8_t reg, uint8_t data) {
	I2C_Start();
	I2C_Write(MPU9250_ADDRESS << 1);
	I2C_Write(reg);
	I2C_Write(data);
	I2C_Stop();
}

uint8_t MPU9250_Read(uint8_t reg) {
	uint8_t data;
	I2C_Start();
	I2C_Write((MPU9250_ADDRESS << 1) | 0x00); // Write address
	I2C_Write(reg);
	I2C_Start();
	I2C_Write((MPU9250_ADDRESS << 1) | 0x01); // Read address
	data = I2C_ReadNack();
	I2C_Stop();
	return data;
}

void MPU9250_Init() {
	// Wake up MPU9250
	MPU9250_Write(0x6B, 0x00);
	_delay_ms(100);
	// Set accelerometer configuration
	MPU9250_Write(0x1C, 0x00); // ±2g
	// Set gyroscope configuration
	MPU9250_Write(0x1B, 0x18); // ±2000 degrees/s
	// Enable bypass mode
	MPU9250_Write(0x37, 0x02);
}

void Read_Accel_Gyro(int16_t* accel, int16_t* gyro) {
	uint8_t rawData[14];
	for (int i = 0; i < 14; i++) {
		rawData[i] = MPU9250_Read(0x3B + i);
	}
	// Accel data
	accel[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	accel[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	accel[2] = ((int16_t)rawData[4] << 8) | rawData[5];

	// Gyro raw data
	gyro[0] = ((int16_t)rawData[8] << 8) | rawData[9];
	gyro[1] = ((int16_t)rawData[10] << 8) | rawData[11];
	gyro[2] = ((int16_t)rawData[12] << 8) | rawData[13];

	// Gyro Calibrated data
	gyro[0] -= 4; //gyroBias[0];
	gyro[1] -= 64; //gyroBias[1];
	gyro[2] -= 35; //gyroBias[2];
}

void Calibrate_Gyro(int16_t* gyroBias) {
	int32_t gyroSum[3] = {0, 0, 0};
	int16_t gyro[3];

	for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
		Read_Accel_Gyro(NULL, gyro);
		gyroSum[0] += gyro[0];
		gyroSum[1] += gyro[1];
		gyroSum[2] += gyro[2];

		UART0_transmit_int16(i);
		UART0_transmit('\n');
		_delay_ms(10); // Small delay between samples
	}

	// Calculate the average bias
	gyroBias[0] = gyroSum[0] / GYRO_CALIBRATION_SAMPLES;
	gyroBias[1] = gyroSum[1] / GYRO_CALIBRATION_SAMPLES;
	gyroBias[2] = gyroSum[2] / GYRO_CALIBRATION_SAMPLES;
}

// AK8963 Functions
void AK8963_Write(uint8_t reg, uint8_t data) {
	I2C_Start();
	I2C_Write(AK8963_ADDRESS << 1);
	I2C_Write(reg);
	I2C_Write(data);
	I2C_Stop();
}

uint8_t AK8963_Read(uint8_t reg) {
	uint8_t data;
	I2C_Start();
	I2C_Write((AK8963_ADDRESS << 1) | 0x00); // Write address
	I2C_Write(reg);
	I2C_Start();
	I2C_Write((AK8963_ADDRESS << 1) | 0x01); // Read address
	data = I2C_ReadNack();
	I2C_Stop();
	return data;
}

void AK8963_Init() {
	// Power down AK8963
	AK8963_Write(0x0A, 0x00);
	_delay_ms(10);
	// Enter Fuse ROM access mode
	AK8963_Write(0x0A, 0x0F);
	_delay_ms(10);
	// set continuous measurement mode 2 (100Hz, 16bit)
	AK8963_Write(0x0A, 0x16);
	_delay_ms(20);
}

void Read_Magnetometer(int16_t* mag) {
	uint8_t rawData[6];
	int16_t cal_mag[3];
	
	AK8963_Read(0x02);	// read ST1
	for (int i = 0; i < 6; i++) {
		rawData[i] = AK8963_Read(0x03 + i);
	}
	AK8963_Read(0x09);	// read ST2
	
	// Magnetometer data
	cal_mag[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	cal_mag[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	cal_mag[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	
	cal_mag[0] -= 210;
	cal_mag[1] -= 200;
	cal_mag[2] -= -240;
	
	mag[0] = (int16_t)(0.999*cal_mag[0]  - 0.0082*cal_mag[1] - 0.0016*cal_mag[2]);
	mag[1] = (int16_t)(-0.0082*cal_mag[0]  + 0.9988*cal_mag[1] + 0.0481*cal_mag[2]);
	mag[2] = (int16_t)(-0.0016*cal_mag[0]  + 0.0481*cal_mag[1] + 1.0045*cal_mag[2]);
	 
	
}

void Calibrate_AK8963(float* magBias, float* magScale) {
	uint16_t ii = 0, sample_count = 1000;
	int32_t mag_temp[3];
	int32_t mag_max[3] = {-32768, -32768, -32768};
	int32_t mag_min[3] = {32767, 32767, 32767};
	int16_t mag[3];

	for (ii = 0; ii < sample_count; ii++) {
		Read_Magnetometer(mag);
		for (int jj = 0; jj < 3; jj++) {
			if (mag[jj] > mag_max[jj]) mag_max[jj] = mag[jj];
			if (mag[jj] < mag_min[jj]) mag_min[jj] = mag[jj];
		}
		
		//if (ii%10==0) UART1_transmit_int16(ii);
		_delay_ms(12); // At least 100 Hz
	}

	// Get hard iron correction
	magBias[0] = (mag_max[0] + mag_min[0]) / 2.0f; // X-axis
	magBias[1] = (mag_max[1] + mag_min[1]) / 2.0f; // Y-axis
	magBias[2] = (mag_max[2] + mag_min[2]) / 2.0f; // Z-axis

	// Get soft iron correction
	magScale[0] = (mag_max[0] - mag_min[0]) / 2.0f; // X-axis
	magScale[1] = (mag_max[1] - mag_min[1]) / 2.0f; // Y-axis
	magScale[2] = (mag_max[2] - mag_min[2]) / 2.0f; // Z-axis

	float avg_rad = magScale[0] + magScale[1] + magScale[2];
	avg_rad /= 3.0f;

	magScale[0] = avg_rad / magScale[0];
	magScale[1] = avg_rad / magScale[1];
	magScale[2] = avg_rad / magScale[2];
}