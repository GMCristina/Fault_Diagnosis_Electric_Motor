/*
 * ADE9000_API.h
 *
 *  Created on: Feb 14, 2022
 *      Author: M.Cristina Giannini
 */

#ifndef INC_ADE9000_API_H_
#define INC_ADE9000_API_H_

#include "ADE9000RegMap.h"
#include "main.h"
#include <stdio.h>

#define TIMEOUT_SPI 100
//SPI with 16 bit data frame
#define SIZE_16 1
#define BURST_READ_N 256
#define WAVEFORM_BUFFER_START_ADDR 0x800


extern SPI_HandleTypeDef hspi3;

union ADE_DATA_32{
	uint8_t data_8[4];
	uint16_t data_16[2];
	uint32_t  data_32;
};

union ADE_DATA_16{
	uint8_t data_8[2];
	uint16_t data_16;
};

uint16_t ADE9000_SPI_Read_16(uint16_t Address);
uint32_t ADE9000_SPI_Read_32(uint16_t Address);


void ADE9000_SPI_Write_16(uint16_t Address, uint16_t Data);
void ADE9000_SPI_Write_32(uint16_t Address, uint32_t Data);

void ADE9000_Setup(void);
void ADE9000_Power(void);
void Start_Waveform_Buffer(void);

void test_read_write_reg(void);

void ADE9000_SPI_Burst_Read_one_ch(uint16_t Address, uint16_t n, int32_t* data);
void ADE9000_SPI_Burst_Read_two_ch(uint16_t Address, uint16_t n, int32_t* i, int32_t* v);
void ADE9000_SPI_Burst_Read_all(uint16_t Address, uint16_t n, int32_t* ia, int32_t* ib, int32_t* ic, int32_t* in, int32_t* va, int32_t* vb, int32_t* vc);
void ADE9000_Conv_ADC(int32_t* data, uint32_t n);



#endif /* INC_ADE9000_API_H_ */
