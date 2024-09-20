/*
 * PID.c
 *
 * Created: 2024-07-24 오후 8:45:15
 *  Author: stu11
 */
#include "PID.h"


void servo_init() {
	DDRB |= (1<<PB5);
	// 타이머1 설정 (Fast PWM 모드, 50Hz 주파수 생성)
	TCCR1A |= (1 << WGM11) | (1 << COM1A1); // Fast PWM, 비반전 모드
	TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler 8
	
	ICR1 = 39999;  // 50Hz 주기를 위한 TOP 값 (16MHz / (8 * 50Hz)) - 1
}


int16_t PID_control(float target, float current) {
	static float previous_error = 0;
	static float integral = 0;

	float error = target - current;
	integral += error;
	float derivative = error - previous_error;

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
