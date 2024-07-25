/*
 * UART.h
 *
 * Created: 2024-07-20 오전 12:03:11
 *  Author: stu11
 */ 
#ifndef SERIAL_H
#define SERIAL_H

#include <avr/io.h>
#include <stdlib.h>

// UART 초기화 함수
void UART0_init(unsigned long baud);
void UART1_init(unsigned long baud);

// 데이터 송신 함수
void UART0_transmit(unsigned char data);
void UART1_transmit(unsigned char data);

// 데이터 수신 함수
unsigned char UART0_receive(void);
unsigned char UART1_receive(void);

// 문자열 송신 함수
void UART0_transmit_string(const char* str);
void UART1_transmit_string(const char* str);
void UART0_transmit_string_LF(const char* str);
void UART1_transmit_string_LF(const char* str);

// 문자열 수신 함수 (특정 종료 문자까지 수신)
void UART0_receive_string(char* buffer, unsigned char terminator);
void UART1_receive_string(char* buffer, unsigned char terminator);

// int16_t 변수를 ASCII로 변환하여 UART0으로 송신하는 함수
void UART0_transmit_int16(int16_t value);
void UART1_transmit_int16(int16_t value);

// I2C Functions
void I2C_Init();
void I2C_Start();
void I2C_Stop();
void I2C_Write(uint8_t data);
uint8_t I2C_ReadAck();
uint8_t I2C_ReadNack();


#endif // SERIAL_H
