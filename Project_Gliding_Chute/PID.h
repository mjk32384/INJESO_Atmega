/*
 * PID.h
 *
 * Created: 2024-07-24 오후 9:09:25
 *  Author: stu11
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define Kp 1.0
#define Ki 0.1
#define Kd 0.01

#define MIN_WIDTH 700	//usec
#define MAX_WIDTH 2300	//usec
#define NEUTRAL_WIDTH (MAX_WIDTH+MIN_WIDTH)/2

void servo_init();

int16_t PID_control(int16_t target, int16_t current);

void servo_control(int16_t control_value);
