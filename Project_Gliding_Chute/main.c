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
#include "PID.h"
#include "guidance.h"
#include "SPI.h"

#define BUFFER_SIZE 128
#define SAMPLING_RATE 64

char uart0_buffer[BUFFER_SIZE];
char uart1_buffer[BUFFER_SIZE];
char SPI_txbuffer[SPI_BUFFER_SIZE+1];
char SPI_rxbuffer[SPI_BUFFER_SIZE+1];


volatile uint8_t flag = 0;  // 플래그 비트
volatile uint8_t print_flag = 0;  // 플래그 비트

// 타이머 0 초기화 함수
void timer0_init() {
	// CTC 모드 설정 (WGM01 = 1, WGM00 = 0)
	TCCR0 = (1 << WGM01);
	// 출력 비교 값 설정
	OCR0 = 196;  // 16MHz 클럭, 1024 프리스케일러 사용 시 79.719Hz 주기
	// 프리스케일러 1024 설정 (CS02 = 1, CS01 = 0, CS00 = 1)
	TCCR0 |= (1 << CS02) | (1 << CS00);
	// 출력 비교 인터럽트 허용
	TIMSK |= (1 << OCIE0);
	
	TCNT0 = 0; // 카운터값 초기화
	
}


ISR(TIMER0_COMP_vect) {
	flag = 1;  // 플래그 비트 설정
}


int main(void) {
	int16_t accel[3], gyro[3], mag[3];
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	int16_t pressure = 0;
	float v[3] = {1.0f, 0.0f, 0.0f};  // IMU 좌표계의 x축을 위성의 heading 벡터로 정함
	float current_heading;	float desired_heading;
	int16_t servo_input=0;
	int32_t lat=0; int32_t lon=0;
	
	
	UART0_init();
	I2C_Init();
	MPU9250_Init();
	AK8963_Init();
	SPI_MasterInit();
	servo_init();
	
	servo_control(-500);
	_delay_ms(250);
	servo_control(500);
	_delay_ms(250);
	servo_control(0);
	_delay_ms(500);
	
	//UART0_transmit_string_LF("Start initiation!!!");
	
	//UART0_transmit_string_LF("Complete initiation!!!");
	
	sei();
	Read_Accel_Gyro(accel, gyro);
	Read_Magnetometer(mag);
	float ax = accel[1]; //  ∵ ±2g 범위에서 16bit 해상도로 측정 -> [m/s^2]
	float ay = accel[0]; //  x, y 뒤바뀌고 z에는 - 붙은 이유 -> mpu9250 좌표계랑 ak8963 좌표계가 달라서 이를 ak8963 기준으로 바꿔줌
	float az = - accel[2]; //
	float gx = gyro[1] * (2000.0 / 32768.0) * (M_PI / 180.0); // ∵ ±2000deg/s 범위에서 16bit 해상도로 측정 -> [rad/s]
	float gy = gyro[0] * (2000.0 / 32768.0) * (M_PI / 180.0);
	float gz = - gyro[2] * (2000.0 / 32768.0) * (M_PI / 180.0);
	float mx = mag[0]; //  ∵ ±4912uT 범위에서 16bit 해상도로 측정 -> [uT]
	float my = mag[1]; // 
	float mz = mag[2]; //
	AHRS_Init(q, ax, ay, az, mx, my, mz);
	
	timer0_init();
	while (1) {
		if(flag==1) {
			Read_Accel_Gyro(accel, gyro);
			Read_Magnetometer(mag);
		
			ax = accel[1]; // ∵ ±2g 범위에서 16bit 해상도로 측정 -> [m/s^2]
			ay = accel[0];
			az = - accel[2];
			gx = gyro[1] * (2000.0 / 32768.0) * (M_PI / 180.0); // ∵ ±2000deg/s 범위에서 16bit 해상도로 측정 -> [rad/s]
			gy = gyro[0] * (2000.0 / 32768.0) * (M_PI / 180.0);
			gz = - gyro[2] * (2000.0 / 32768.0) * (M_PI / 180.0);
			mx = mag[0]; // ∵ ±4912uT 범위에서 16bit 해상도로 측정 -> [uT]
			my = mag[1];
			mz = mag[2];
						
			MadgwickQuaternionUpdate(q, ax, ay, az, gx, gy, gz, mx, my, mz);
			current_heading = Theta1(q, v);
			desired_heading = 0;
			
			servo_input = PID_control(desired_heading, current_heading);
			servo_control(servo_input);
			
			if(print_flag++ == 9){
				pressure = 1013;
				//split_master_data(q, servo_input, pressure, SPI_txbuffer);
				//SPI_MasterTransfer(SPI_txbuffer, SPI_rxbuffer);
				//concat_slave_data(&lat, &lon, SPI_rxbuffer);
				UART0_transmit_int16( (int16_t)(mag[0]) ); UART0_transmit(',');
				UART0_transmit_int16( (int16_t)(mag[1]) ); UART0_transmit(',');
				UART0_transmit_int16( (int16_t)(mag[2]) ); UART0_transmit('\n');
				//UART0_transmit_int16(servo_input);UART0_transmit('\n');
				print_flag = 0;
				
			}
			
			flag = 0;
		}
		
	}
}
