/*
 * Project_Gliding_Chute.c
 *
 * Created: 2024-07-24 오후 5:29:08
 * Author : stu11
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include "serial.h"
#include "sensor.h"
#include "AHRS.h"

#define BUFFER_SIZE 128
#define SAMPLING_RATE 64

char uart0_buffer[BUFFER_SIZE];
char uart1_buffer[BUFFER_SIZE];


volatile uint8_t flag = 0;  // 플래그 비트

volatile uint32_t overflow_count = 0;

// 타이머3 초기화 함수
void timer3_init() {
	TCCR3A = 0; // 타이머 모드 설정: Normal mode
	TCCR3B = (1 << CS31); // 프리스케일러 설정: clk/8
	TCNT3 = 0; // 타이머 카운터 초기화
	TIMSK = (1 << TOIE3); // 타이머 오버플로우 인터럽트 활성화
}


// 타이머 0 초기화 함수
void timer0_init() {
	// CTC 모드 설정 (WGM01 = 1, WGM00 = 0)
	TCCR0 = (1 << WGM01);
	// 출력 비교 값 설정 (64Hz 주기 생성)
	OCR0 = 243;  // 16MHz 클럭, 1024 프리스케일러 사용 시 128Hz 주기
	// 프리스케일러 1024 설정 (CS02 = 1, CS01 = 0, CS00 = 1)
	TCCR0 |= (1 << CS02) | (1 << CS00);
	// 출력 비교 인터럽트 허용
	TIMSK |= (1 << OCIE0);
	
}

ISR(TIMER0_COMP_vect) {
	flag = 1;  // 플래그 비트 설정
}
ISR(USART0_RX_vect) {
	
	UART0_receive_string(uart0_buffer, '\n');
}
ISR(USART1_RX_vect) {

	UART1_receive_string(uart1_buffer, '\n');
	
}

ISR(TIMER3_OVF_vect) {
	overflow_count++;
}



int16_t ticks_to_microseconds(uint32_t ticks) {
	// 프리스케일러가 8이므로 한 틱은 1/(F_CPU/8) 초
	// F_CPU가 16MHz인 경우, 한 틱은 0.5 마이크로초
	uint32_t microseconds = ticks * 0.5;
	// int16_t로 변환, 오버플로우 방지
	if (microseconds > INT16_MAX) {
		return INT16_MAX; // 최대값으로 제한
	}
	return (int16_t)microseconds;
}

/*
int main(void) {
	
	int16_t accel[3], gyro[3], mag[3];
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	
		
	uint16_t start_time, end_time;
	uint32_t elapsed_ticks;
	int16_t elapsed_time_us;
	
	// 타이머 초기화
	UART1_init(57600);
	UART1_transmit_string_LF("Start initiation!!!");
	I2C_Init();
	MPU9250_Init();
	AK8963_Init();
	UART1_transmit_string_LF("Complete initiation!!!");
	
	Read_Accel_Gyro(accel, gyro);UART1_transmit('a');
	Read_Magnetometer(mag);UART1_transmit('b');
	float ax = accel[1]; // * 9.81 * 2.0 / 32768.0; // ∵ ±2g 범위에서 16bit 해상도로 측정 -> [m/s^2]
	float ay = accel[0]; // * 9.81 * 2.0 / 32768.0; // x, y 뒤바뀌고 z에는 - 붙은 이유 -> mpu9250 좌표계랑 ak8963 좌표계가 달라서 이를 ak8963 기준으로 바꿔줌
	float az = - accel[2]; // * 9.81 * 2.0 / 32768.0;
	float gx = gyro[1] * (250.0 / 32768.0) * (M_PI / 180.0); // ∵ ±250deg/s 범위에서 16bit 해상도로 측정 -> [rad/s]
	float gy = gyro[0] * (250.0 / 32768.0) * (M_PI / 180.0);
	float gz = - gyro[2] * (250.0 / 32768.0) * (M_PI / 180.0);
	float mx = mag[0]; // * 4912 / 32760.0; // ∵ ±4912uT 범위에서 16bit 해상도로 측정 -> [uT]
	float my = mag[1]; // * 4912 / 32760.0;
	float mz = mag[2]; // * 4912 / 32760.0;
	AHRS_Init(q, ax, ay, az, mx, my, mz);
	UART1_transmit('b');
	
	
	
	
	// 함수 실행 전 카운터 값 저장
	timer3_init();
	//sei();
	start_time = TCNT3;


	// 측정할 함수 실행
	
	Read_Accel_Gyro(accel, gyro);
	Read_Magnetometer(mag);
	
	ax = accel[1]; // ∵ ±2g 범위에서 16bit 해상도로 측정 -> [m/s^2]
	ay = accel[0]; //
	az = - accel[2]; //
	gx = gyro[1] * (250.0 / 32768.0) * (M_PI / 180.0); // ∵ ±250deg/s 범위에서 16bit 해상도로 측정 -> [rad/s]
	gy = gyro[0] * (250.0 / 32768.0) * (M_PI / 180.0);
	gz = - gyro[2] * (250.0 / 32768.0) * (M_PI / 180.0);
	mx = mag[0]; // ∵ ±4912uT 범위에서 16bit 해상도로 측정 -> [uT]
	my = mag[1]; //
	mz = mag[2]; //
	
	MadgwickQuaternionUpdate(q, ax, ay, az, gx, gy, gz, mx, my, mz);
	
	
	

	// 함수 실행 후 카운터 값 저장
	end_time = TCNT3;

	// 실행 시간 계산
	if (end_time >= start_time) {
		elapsed_ticks = (end_time - start_time) + (overflow_count * 0x10000);
		} else {
		elapsed_ticks = (0xFFFF - start_time + end_time + 1) + (overflow_count * 0x10000);
	}

	// 타이머 틱 수를 마이크로초 단위로 변환
	elapsed_time_us = ticks_to_microseconds(elapsed_ticks);

	// 여기서 elapsed_time_us는 함수 실행 시간 (마이크로초 단위)
	UART1_transmit_int16(elapsed_time_us);
	while (1) {
		// 메인 루프
	}

	return 0;
}

*/


int main(void) {
	int16_t accel[3], gyro[3], mag[3];
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	int index = 0;
	UART1_init(57600);
	
	//UART1_transmit_string_LF("Start initiation!!!");
	I2C_Init();
	MPU9250_Init();
	AK8963_Init();
	
	//UART1_transmit_string_LF("Complete initiation!!!");
	//timer0_init();

	
	sei();
	 
	Read_Accel_Gyro(accel, gyro);
	UART1_transmit_int16(accel[0]); UART1_transmit_string(",");
	UART1_transmit_int16(accel[1]); UART1_transmit_string(",");
	UART1_transmit_int16(accel[2]); UART1_transmit('\n');
	Read_Magnetometer(mag);
	float ax = accel[1]; // * 9.81 * 2.0 / 32768.0; // ∵ ±2g 범위에서 16bit 해상도로 측정 -> [m/s^2]
	float ay = accel[0]; // * 9.81 * 2.0 / 32768.0; // x, y 뒤바뀌고 z에는 - 붙은 이유 -> mpu9250 좌표계랑 ak8963 좌표계가 달라서 이를 ak8963 기준으로 바꿔줌
	float az = - accel[2]; // * 9.81 * 2.0 / 32768.0;
	float gx = gyro[1] * (250.0 / 32768.0) * (M_PI / 180.0); // ∵ ±250deg/s 범위에서 16bit 해상도로 측정 -> [rad/s]
	float gy = gyro[0] * (250.0 / 32768.0) * (M_PI / 180.0);
	float gz = - gyro[2] * (250.0 / 32768.0) * (M_PI / 180.0);
	float mx = mag[0]; // * 4912 / 32760.0; // ∵ ±4912uT 범위에서 16bit 해상도로 측정 -> [uT]
	float my = mag[1]; // * 4912 / 32760.0; 
	float mz = mag[2]; // * 4912 / 32760.0;
	AHRS_Init(q, ax, ay, az, mx, my, mz);
	
	while (1) {
		Read_Accel_Gyro(accel, gyro);
		Read_Magnetometer(mag);
			
		ax = accel[1] ;// * 9.81 * 2.0 / 32768.0; // ∵ ±2g 범위에서 16bit 해상도로 측정 -> [m/s^2]
		ay = accel[0] ;// * 9.81 * 2.0 / 32768.0;
		az = - accel[2];// * 9.81 * 2.0 / 32768.0;
		gx = gyro[1] * (250.0 / 32768.0) * (M_PI / 180.0); // ∵ ±250deg/s 범위에서 16bit 해상도로 측정 -> [rad/s]
		gy = gyro[0] * (250.0 / 32768.0) * (M_PI / 180.0);
		gz = - gyro[2] * (250.0 / 32768.0) * (M_PI / 180.0);
		mx = mag[0] ;// * 4912 / 32760.0; // ∵ ±4912uT 범위에서 16bit 해상도로 측정 -> [uT]
		my = mag[1] ;//  * 4912 / 32760.0; 
		mz = mag[2] ;// * 4912 / 32760.0;
						
		MadgwickQuaternionUpdate(q, ax, ay, az, gx, gy, gz, mx, my, mz);
		
		UART1_transmit_int16(accel[0]); UART1_transmit_string(",");
		UART1_transmit_int16(accel[1]); UART1_transmit_string(",");
		UART1_transmit_int16(accel[2]); UART1_transmit('\n');


	}
}

			