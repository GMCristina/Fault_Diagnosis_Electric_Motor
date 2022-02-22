/*
 * ADE9000_API.c
 *
 *  Created on: Feb 14, 2022
 *      Author: M.Cristina Giannini
 */

#include "ADE9000_API.h"
#include "main.h"

void ADE9000_Power(void){
	   //PM1
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	   //RESET
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

	   //funzione RESET (serve???)
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	   HAL_Delay(50);
	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	   HAL_Delay(1000);

	   HAL_Delay(1000);
}

uint16_t ADE9000_SPI_Read_16(uint16_t Address){

	union ADE_DATA_16 data;
	union ADE_DATA_16 addr;
	HAL_StatusTypeDef ret;

	//Indirizzo per lettura
	addr.data_16 = (((Address <<4) & 0xFFF0)+8);

	//CS
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Invia indirizzo
	ret = HAL_SPI_Transmit(&hspi3,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//riceve dati
	data.data_16 = 3;
	ret = HAL_SPI_Receive(&hspi3,data.data_8,SIZE_16,TIMEOUT_SPI);

	//CS
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);

	return data.data_16;
}


uint32_t ADE9000_SPI_Read_32(uint16_t Address){

	union ADE_DATA_32 data;
	union ADE_DATA_16 addr;
	HAL_StatusTypeDef ret;

	//Indirizzo per lettura
	addr.data_16 = (((Address <<4) & 0xFFF0)+8);

	//CS
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Invia indirizzo
	ret = HAL_SPI_Transmit(&hspi3,addr.data_8,SIZE_32,TIMEOUT_SPI);

	//riceve dati
	ret = HAL_SPI_Receive(&hspi3,data.data_8,SIZE_32,TIMEOUT_SPI);

	//CS
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);

	return data.data_32;
}

void ADE9000_SPI_Write_16(uint16_t Address, uint16_t Data){

	union ADE_DATA_16 addr;
	union ADE_DATA_16 data;
	HAL_StatusTypeDef ret;

	//Indirizzo per lettura
	addr.data_16 = ((Address <<4) & 0xFFF0);

	//CS
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Invia indirizzo
	ret = HAL_SPI_Transmit(&hspi3,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//invia dati
	data.data_16 = Data;
	ret = HAL_SPI_Transmit(&hspi3,data.data_8,SIZE_16,TIMEOUT_SPI);

	//CS
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);

}

void ADE9000_SPI_Write_32(uint16_t Address, uint32_t Data){

	union ADE_DATA_16 addr;
	union ADE_DATA_32 data;
	HAL_StatusTypeDef ret;

	//Indirizzo per lettura
	addr.data_16 = ((Address <<4) & 0xFFF0);

	//CS
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Invia indirizzo
	ret = HAL_SPI_Transmit(&hspi3,addr.data_8,SIZE_32,TIMEOUT_SPI);

	//invia dati
	data.data_32 = Data;
	ret = HAL_SPI_Transmit(&hspi3,data.data_8,SIZE_32,TIMEOUT_SPI);

	//CS
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);

}

void ADE9000_Setup(){
	union ADE_DATA_32 value_reg_32;
	union ADE_DATA_16 value_reg_16;

	value_reg_16.data_16 = 0x0001;
	ADE9000_SPI_Write_16(ADDR_RUN, value_reg_16.data_16);
}
