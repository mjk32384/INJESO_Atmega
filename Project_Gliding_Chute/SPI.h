/*
 * SPI.h
 *
 * Created: 2024-08-27 오후 8:27:04
 *  Author: stu11
 */ 

#include <avr/io.h>
#include <stdlib.h>

#define SPI_BUFFER_SIZE 12


void SPI_MasterInit(void);
void SPI_SlaveInit(void);
void SPI_MasterTransfer(char* SPI_txbuffer, char* SPI_rxbuffer);

void split_master_data(float qtn[4], int16_t serv_in, int16_t atm, char spi_buffer[SPI_BUFFER_SIZE+1]);
void concat_master_data(int16_t qtn[4], int16_t serv_in[1], int16_t atm[1], char spi_buffer[SPI_BUFFER_SIZE+1]);
void split_slave_data(int32_t lat, int32_t lon, char spi_buffer[SPI_BUFFER_SIZE+1]);
void concat_slave_data(int32_t lat[1], int32_t lon[1], char spi_buffer[SPI_BUFFER_SIZE+1]);