/*
 * PID.c
 *
 * Created: 2024-07-24 오후 8:45:15
 *  Author: stu11
 */
#include "PID.h"

void PWM_init() {
	// Fast PWM mode, non-inverting, prescaler 64
	TCCR1A = (1<<COM1A1) | (1<<WGM11);
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11) | (1<<CS10);
	ICR1 = 49999; // fPWM = 50Hz (20ms period)
	DDRB |= (1<<PB5); // PB5 (OC1A) as output
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
	// 서보모터는 1ms에서 2ms 사이의 펄스 신호를 필요로 합니다.
	// 50Hz PWM에서 1ms는 1000us, 2ms는 2000us입니다.
	// ICR1 = 49999일 때, 1ms = 1000 / (20000/49999), 2ms = 2000 / (20000/49999)
	uint16_t pulse_width = 1000 + control_value; // control_value가 -500 ~ 500 사이의 값이라고 가정
	
	if (pulse_width < 1000) pulse_width = 1000;
	if (pulse_width > 2000) pulse_width = 2000;
	
	OCR1A = (pulse_width * 49999) / 20000;
}
