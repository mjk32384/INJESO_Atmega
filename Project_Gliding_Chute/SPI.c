/*
 * SPI.c
 *
 * Created: 2024-08-27 오후 8:26:26
 *  Author: stu11
 */ 
#include "SPI.h"

void SPI_MasterInit(void) {
	// MOSI, SCK, SS를 출력으로 설정
	DDRB |= (1 << PB2) | (1 << PB1) | (1 << PB0);
	// MISO는 입력으로 설정
	DDRB &= ~(1 << PB3);
	
	// SPI 활성화, 마스터 모드, 클럭 설정 (fosc/16)
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
	PORTB |= (1<<PB0);
}

void SPI_SlaveInit(void) {
	// MISO를 출력으로 설정
	DDRB = (1 << PB3);
	// MOSI, SCK, SS를 입력으로 설정
	DDRB &= ~((1 << PB2) | (1 << PB1) | (1 << PB0));
	
	// SPI 활성화, 슬레이브 모드, SPI 인터럽트 활성화
	SPCR = (1 << SPE) | (1<<SPIE);
}


void SPI_MasterTransfer(char* SPI_txbuffer, char* SPI_rxbuffer){
	PORTB &= ~(1<<PB0);		// Chip Select 활성화
	
	for (int i = 0; i < SPI_BUFFER_SIZE; i++) {
		SPDR = SPI_txbuffer[i];
		while (!(SPSR & (1 << SPIF)));  // 전송 완료 대기
		SPI_rxbuffer[i] = SPDR;
	}
	
	PORTB |= (1 << PB0);	// Chip Select 비활성화
}

void split_master_data(float q[4], int16_t serv_in, int16_t atm, char spi_buffer[12]) {
	// Load the {spi_buffer} with {qtn}, {atm}, and {serv_in} data
	int16_t qtn[4];
	
	qtn[0] = (int16_t)(10000*q[0]);
	qtn[1] = (int16_t)(10000*q[1]);
	qtn[2] = (int16_t)(10000*q[2]);
	qtn[3] = (int16_t)(10000*q[3]);
	
	for (int i=0; i<4; i++) {
		spi_buffer[2*i] = (qtn[i] >> 8) & 0xFF;
		spi_buffer[2*i+1] = qtn[i] & 0xFF;
	}
	spi_buffer[8] = (serv_in >> 8) & 0xFF;
	spi_buffer[9] = serv_in & 0xFF;
	spi_buffer[10] = (atm >> 8) & 0xFF;
	spi_buffer[11] = atm & 0xFF;
}

void concat_master_data(int16_t qtn[4], int16_t serv_in[1], int16_t atm[1], char spi_buffer[12]) {
	// Concatenate the data from given {spi_buffer}
	for (int i = 0; i < 4; i++) {
		qtn[i] = (int16_t)(spi_buffer[2*i] << 8) | spi_buffer[2*i+1];
	}
	serv_in[0] = (int16_t)(spi_buffer[8] << 8) | spi_buffer[9];
	atm[0] = (int16_t)(spi_buffer[10] << 8) | spi_buffer[11];
}

void split_slave_data(int32_t lat, int32_t lon, char spi_buffer[12]) {
	// Load the {spi_buffer} with {lat}, {lon}
	spi_buffer[0] = 0xFF;   // garbage
	spi_buffer[1] = 0xFF; // garbage

	// Latitude (4 bytes)
	spi_buffer[2] = (lat >> 24) & 0xFF;
	spi_buffer[3] = (lat >> 16) & 0xFF;
	spi_buffer[4] = (lat >> 8) & 0xFF;
	spi_buffer[5] = lat & 0xFF;

	// Longitude (4 bytes)
	spi_buffer[6] = (lon >> 24) & 0xFF;
	spi_buffer[7] = (lon >> 16) & 0xFF;
	spi_buffer[8] = (lon >> 8) & 0xFF;
	spi_buffer[9] = lon & 0xFF;

	spi_buffer[10] = 0xFF; // garbage
	spi_buffer[11] = 0xFF; // garbage

}

void concat_slave_data(int32_t lat[1], int32_t lon[1], char spi_buffer[12]) {
	// Concatenate the data from given {spi_buffer}
	lat[0] = (int32_t)(spi_buffer[2] << 24) | (spi_buffer[3] << 16) | (spi_buffer[4] << 8) | spi_buffer[5];
	lon[0] = (int32_t)(spi_buffer[6] << 24) | (spi_buffer[7] << 16) | (spi_buffer[8] << 8) | spi_buffer[9];
}