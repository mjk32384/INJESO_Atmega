/*
 * PID.h
 *
 * Created: 2024-07-24 오후 9:09:25
 *  Author: stu11
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define Kp 200
#define Ki 0
#define Kd 0

#define MIN_WIDTH 1000	//usec
#define MAX_WIDTH 2000	//usec
#define NEUTRAL_WIDTH 1500

void servo_init();

int16_t PID_control(float target, float current);

void servo_control(int16_t control_value);
