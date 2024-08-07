/*
 * CFile1.c
 *
 * Created: 2024-07-20 오전 12:02:58
 *  Author: stu11
 */ 
#include "serial.h"

void UART0_init(void) {
	UBRR0H = 0x00;                     //115,200 baudrate fixed
	UBRR0L = 16;
	UCSR0A |= _BV(U2X0);               //2배속 모드
	// 비동기, 8비트 데이터, 패리티 없음, 1비트 정지 비트 모드
	UCSR0C |= 0x06;
	
	UCSR0B |= _BV(RXEN0);               //송수신 가능
	UCSR0B |= _BV(TXEN0);
}

void UART0_transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

unsigned char UART0_receive(void) {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void UART0_transmit_string(const char* str) {
	while (*str) {
		UART0_transmit(*str++);
	}
}

void UART0_transmit_string_LF(const char* str) {
	while (*str) {
		UART0_transmit(*str++);
	}
	UART0_transmit('\n');
}

void UART0_receive_string(char* buffer, unsigned char terminator) {
	unsigned char receivedChar;
	while (1) {
		receivedChar = UART0_receive();
		if (receivedChar == terminator) {
			*buffer = '\0';
			break;
		}
		*buffer++ = receivedChar;
	}
}

void UART0_transmit_int16(int16_t value) {
	char buffer[7];  // int16_t는 -32768부터 32767까지 표현 가능하므로, 6자리 숫자 + null 문자
	itoa(value, buffer, 10);  // 정수를 문자열로 변환
	UART0_transmit_string(buffer);  // 변환된 문자열을 UART0으로 송신
}

void UART1_init(void) {
	UBRR1H = 0x00;                     //115,200 baudrate fixed
	UBRR1L = 16;
	UCSR1A |= _BV(U2X1);               //2배속 모드
	// 비동기, 8비트 데이터, 패리티 없음, 1비트 정지 비트 모드
	UCSR1C |= 0x06;
	
	UCSR1B |= _BV(RXEN1);               //송수신 가능
	UCSR1B |= _BV(TXEN1);
}

void UART1_transmit(unsigned char data) {
	while (!(UCSR1A & (1 << UDRE1)));
	UDR1 = data;
}

unsigned char UART1_receive(void) {
	while (!(UCSR1A & (1 << RXC1)));
	return UDR1;
}

void UART1_transmit_string(const char* str) {
	while (*str) {
		UART1_transmit(*str++);
	}
}

void UART1_transmit_string_LF(const char* str) {
	while (*str) {
		UART1_transmit(*str++);
	}
	UART1_transmit('\n');
}

void UART1_receive_string(char* buffer, unsigned char terminator) {
	unsigned char receivedChar;
	while (1) {
		receivedChar = UART1_receive();
		if (receivedChar == terminator) {
			*buffer = '\0';
			break;
		}
		*buffer++ = receivedChar;
	}
}

void UART1_transmit_int16(int16_t value) {
	char buffer[7];  // int16_t는 -32768부터 32767까지 표현 가능하므로, 6자리 숫자 + null 문자
	itoa(value, buffer, 10);  // 정수를 문자열로 변환
	UART1_transmit_string(buffer);  // 변환된 문자열을 UART0으로 송신
}

// I2C Functions
void I2C_Init() {
	TWSR = 0x00; // Prescaler value set to 1
	TWBR = 0x46; // SCL frequency set to 100kHz with 8MHz CPU clock
	TWCR = (1 << TWEN); // Enable TWI
}

void I2C_Start() {
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT); // Send START condition
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT flag set
}

void I2C_Stop() {
	TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT); // Send STOP condition
}

void I2C_Write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT); // Start transmission of data
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT flag set
}

uint8_t I2C_ReadAck() {
	TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA); // Start receiving data with ACK
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT flag set
	return TWDR;
}

uint8_t I2C_ReadNack() {
	TWCR = (1 << TWEN) | (1 << TWINT); // Start receiving data without ACK
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT flag set
	return TWDR;
}
