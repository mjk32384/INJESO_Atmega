
/*
 * AHRS.h
 *
 * Created: 2024-07-24 오후 5:30:54
 *  Author: stu11
 */ 

#ifndef AHRS_H
#define AHRS_H

#define beta 1.6
#define deltat 0.012544

#include <avr/io.h>
#include <util/delay.h>

void Quaternion_rotate(float* q, float v[3], float output[3]);
void Quaternion_multiply(float q1[3], float q2[3], float output[3]);
void AHRS_Init(float* q, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickQuaternionUpdate(float* q, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
#endif