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

#define BUFFER_SIZE 128
#define SAMPLING_RATE 64

char uart0_buffer[BUFFER_SIZE];
char uart1_buffer[BUFFER_SIZE];

volatile uint8_t flag = 0;  // 플래그 비트

// 타이머 0 초기화 함수
void timer0_init() {
	// CTC 모드 설정 (WGM01 = 1, WGM00 = 0)
	TCCR0 = (1 << WGM01);
	// 출력 비교 값 설정 (512Hz 주기 생성)
	OCR0 = 29;  // 16MHz 클럭, 1024 프리스케일러 사용 시 512Hz 주기
	// 프리스케일러 1024 설정 (CS02 = 1, CS01 = 0, CS00 = 1)
	TCCR0 |= (1 << CS02) | (1 << CS00);
	// 출력 비교 인터럽트 허용
	TIMSK |= (1 << OCIE0);
}

ISR(TIMER0_COMP_vect) {
	flag = 1;  // 플래그 비트 설정
}


int main(void) {
	int16_t accel[3], gyro[3], mag[3];
	//float magBias[3], magScale[3];
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	int index = 0;
	UART1_init(57600);
	//UART1_transmit('A');
	//UART1_receive();
	
	//UART1_transmit_string_LF("Start initiation!!!");
	I2C_Init();
	MPU9250_Init();
	AK8963_Init();
	
	//UART1_transmit_string_LF("Complete initiation!!!");
	timer0_init();
	
	sei();
	
	Read_Accel_Gyro(accel, gyro);
	Read_Magnetometer(mag);
	float ax = accel[1] * 9.81 * 2.0 / 32768.0; // ∵ ±2g 범위에서 16bit 해상도로 측정 -> [m/s^2]
	float ay = accel[0] * 9.81 * 2.0 / 32768.0;
	float az = - accel[2] * 9.81 * 2.0 / 32768.0;
	float gx = gyro[1] * (250.0 / 32768.0) * (M_PI / 180.0); // ∵ ±250deg/s 범위에서 16bit 해상도로 측정 -> [rad/s]
	float gy = gyro[0] * (250.0 / 32768.0) * (M_PI / 180.0);
	float gz = - gyro[2] * (250.0 / 32768.0) * (M_PI / 180.0);
	float mx = mag[0] * 4912 / 32760.0; // ∵ ±4912uT 범위에서 16bit 해상도로 측정 -> [uT]
	float my = mag[1] * 4912 / 32760.0; // x, y 뒤바뀌고 z에는 - 붙은 이유 -> mpu9250 좌표계랑 ak8963 좌표계가 달라서 이를 mpu9250 기준으로 바꿔줌
	float mz = mag[2] * 4912 / 32760.0;
	AHRS_Init(q, ax, ay, az, mx, my, mz);
	
	
	//Calibrate_AK8963(magBias, magScale);

	while (1) {
		if (flag) {
			Read_Accel_Gyro(accel, gyro);
			Read_Magnetometer(mag);
			
			ax = accel[1] * 9.81 * 2.0 / 32768.0; // ∵ ±2g 범위에서 16bit 해상도로 측정 -> [m/s^2]
			ay = accel[0] * 9.81 * 2.0 / 32768.0;
			az = - accel[2] * 9.81 * 2.0 / 32768.0;
			gx = gyro[1] * (250.0 / 32768.0) * (M_PI / 180.0); // ∵ ±250deg/s 범위에서 16bit 해상도로 측정 -> [rad/s]
			gy = gyro[0] * (250.0 / 32768.0) * (M_PI / 180.0);
			gz = - gyro[2] * (250.0 / 32768.0) * (M_PI / 180.0);
			mx = mag[0] * 4912 / 32760.0; // ∵ ±4912uT 범위에서 16bit 해상도로 측정 -> [uT]
			my = mag[1] * 4912 / 32760.0; // x, y 뒤바뀌고 z에는 - 붙은 이유 -> mpu9250 좌표계랑 ak8963 좌표계가 달라서 이를 mpu9250 기준으로 바꿔줌
			mz = mag[2] * 4912 / 32760.0;			
			MadgwickQuaternionUpdate(q, ax, ay, az, gx, gy, gz, mx, my, mz);
			
			if (index++ == 20){
				UART1_transmit_int16((int16_t)10000*q[0]); UART1_transmit_string(",");
				UART1_transmit_int16((int16_t)10000*q[1]); UART1_transmit_string(",");
				UART1_transmit_int16((int16_t)10000*q[2]); UART1_transmit_string(",");
				UART1_transmit_int16((int16_t)10000*q[3]); UART1_transmit('\n');
				index = 0;
			}
			
			flag = 0;  // 플래그 비트 클리어
		}
	}
}


/*
int main(void) {
	
	UART1_init(57600);
	UART1_transmit_string_LF("Start!");
	servo_init();
	sei();
	

	while (1) {
		//int16_t control_value = PID_control(target_value, current_value);
		
		UART1_transmit_string_LF("Servo input: -50");
		servo_control(-50);
		_delay_ms(500);
		
		UART1_transmit_string_LF("Servo input: 0");
		servo_control(0);
		_delay_ms(500);
		
		UART1_transmit_string_LF("Servo input: 50");
		servo_control(50);
		_delay_ms(500);
		
	}

	return 0;

}
*/

