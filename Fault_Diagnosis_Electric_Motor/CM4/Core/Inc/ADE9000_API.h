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

#define TIMEOUT_SPI 100
#define SIZE_16 1
#define SIZE_32 2

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



#endif /* INC_ADE9000_API_H_ */
