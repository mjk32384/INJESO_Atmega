/*
 * PID.c
 *
 * Created: 2024-07-24 오후 8:45:15
 *  Author: stu11
 */
#include "PID.h"

void servo_init() {
	DDRB |= (1<<PB5); // PB5 (OC1A) as output
	
	// Fast PWM mode
	TCCR1A = 0xAA;
	TCCR1B = 0x18;;
	
	ICR1 = 10000*2; // fPWM = 100Hz (10ms period)
	
	OCR1A = NEUTRAL_WIDTH*2;	//set servo position to neutral
	
	TCCR1B |= (2<<CS10); // prescaler 8
}


int16_t PID_control(int16_t target, int16_t current) {
	static int16_t previous_error = 0;
	static int16_t integral = 0;

	int16_t error = target - current;
	integral += error;
	int16_t derivative = error - previous_error;

	int16_t output = (Kp * error) + (Ki * integral) + (Kd * derivative);
	previous_error = error;

	return output;
}

void servo_control(int16_t control_value) {
	
	uint16_t pulse_width = NEUTRAL_WIDTH + control_value;
	
	if (pulse_width < MIN_WIDTH) pulse_width = MIN_WIDTH;
	if (pulse_width > MAX_WIDTH) pulse_width = MAX_WIDTH;
	
	OCR1A = pulse_width*2 ;
}
