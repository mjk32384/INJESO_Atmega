
/*
 * AHRS.h
 *
 * Created: 2024-07-24 오후 5:30:54
 *  Author: stu11
 */ 

#ifndef AHRS_H
#define AHRS_H

#include <avr/io.h>
#include <util/delay.h>

#define SAMPLE_FREQ 64.0f // Sample frequency in Hz

void MadgwickAHRSupdate(float* q, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
#endif