/*
 * ADE9000_API.c
 *
 *  Created on: Feb 14, 2022
 *      Author: M.Cristina Giannini
 */

#include "ADE9000_API.h"
#include "main.h"

//power-on sequence
void ADE9000_Power(void){
	//PM1 pin
	//PM1 e PM0 for power mode (PM1=0 for normal mode)
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);

	//RESET pin (è !reset)
	//reset
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(500);
    //no reset
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(500);

    //Check end power-up sequence (RESTDONE)
    uint32_t value_reg_32;
    do {
    	value_reg_32 = ADE9000_SPI_Read_32(ADDR_STATUS1);
    	HAL_Delay(500);
    } while ((value_reg_32 & 0x00010000)==0);
    //Clear IRQ1
    value_reg_32 = value_reg_32 & 0x00010000;
    ADE9000_SPI_Write_32(ADDR_STATUS1,value_reg_32);
}

uint16_t ADE9000_SPI_Read_16(uint16_t Address){

	union ADE_DATA_16 data;
	union ADE_DATA_16 addr;
	HAL_StatusTypeDef ret;

	//Address for read
	// addr|R/W(1/0)|000
	addr.data_16 = (((Address <<4) & 0xFFF0)+8);

	//CS on
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Send address
	ret = HAL_SPI_Transmit(&hspi3,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//Receive data
	ret = HAL_SPI_Receive(&hspi3,data.data_8,SIZE_16,TIMEOUT_SPI);

	//CS off
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);

	return data.data_16;
}


uint32_t ADE9000_SPI_Read_32(uint16_t Address){

	union ADE_DATA_32 data;
	union ADE_DATA_16 addr;
	HAL_StatusTypeDef ret;

	//Address for read
	// addr|R/W(1/0)|000
	addr.data_16 = (((Address <<4) & 0xFFF0)+8);

	//CS on
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Send address
	ret = HAL_SPI_Transmit(&hspi3,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//Receive data
	ret = HAL_SPI_Receive(&hspi3,data.data_8 + 2,SIZE_16,TIMEOUT_SPI);
	ret = HAL_SPI_Receive(&hspi3,data.data_8,SIZE_16,TIMEOUT_SPI);

	//CS off
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);

	return data.data_32;
}

void ADE9000_SPI_Write_16(uint16_t Address, uint16_t Data){

	union ADE_DATA_16 addr;
	union ADE_DATA_16 data;
	HAL_StatusTypeDef ret;

	//Address for write
	// addr|R/W(1/0)|000
	addr.data_16 = ((Address <<4) & 0xFFF0);

	//CS on
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Send address
	ret = HAL_SPI_Transmit(&hspi3,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//Send data
	data.data_16 = Data;
	ret = HAL_SPI_Transmit(&hspi3,data.data_8,SIZE_16,TIMEOUT_SPI);

	//CS off
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);

}

void ADE9000_SPI_Write_32(uint16_t Address, uint32_t Data){

	union ADE_DATA_16 addr;
	union ADE_DATA_32 data;
	HAL_StatusTypeDef ret;

	//Address for write
	// addr|R/W(1/0)|000
	addr.data_16 = ((Address <<4) & 0xFFF0);

	//CS on
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Send address
	ret = HAL_SPI_Transmit(&hspi3,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//Send data
	data.data_32 = Data;
	ret = HAL_SPI_Transmit(&hspi3,data.data_8 +2,SIZE_16,TIMEOUT_SPI);
	ret = HAL_SPI_Transmit(&hspi3,data.data_8,SIZE_16,TIMEOUT_SPI);

	//CS off
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);

}

void ADE9000_Setup(){
	uint32_t value_reg_32;
	uint16_t value_reg_16;

	// ADDR_PGA_GAIN
	value_reg_16 = 0x0000; //gain all channel 1
	ADE9000_SPI_Write_16(ADDR_PGA_GAIN,value_reg_16);

	//CONFIG2
	value_reg_16 = 	0x0C00;			//Default High pass corner frequency of 1.25Hz
	ADE9000_SPI_Write_16(ADDR_CONFIG2,value_reg_16);

	//ACCMODE
	value_reg_16= 0x0000;			//3P4W Wye configuration
	ADE9000_SPI_Write_16(ADDR_ACCMODE,value_reg_16);

	//WFB_CFG
	//Res,Res,Res,IN
	//Res,Res,Source
	//Mode,Fixed/resample,Avvio
	//channel burst
	value_reg_16 = 0x0028; //no IN, sinc4, stop full, fixed rate, stop, solo IA
	ADE9000_SPI_Write_16(ADDR_WFB_CFG ,value_reg_16);

	//WFB_PG_IRQEN
	// 1 bit per pagina: pag15-pag0
	//fa PageFULL in STATUS0
	value_reg_16 = 0x8000; //page 15
	ADE9000_SPI_Write_16(ADDR_WFB_PG_IRQEN,value_reg_16);

	//ADDR_MASK0
	//IRQ0 per full page (bit 17)
	value_reg_32 = 0x00020000;
	ADE9000_SPI_Write_32(ADDR_MASK0,value_reg_32);
	value_reg_32 = ADE9000_SPI_Read_32(ADDR_MASK0);

	//ADDR_MASK1
	//disable all int
	value_reg_32 = 0x00000000;
	ADE9000_SPI_Write_32(ADDR_MASK1,value_reg_32);
	value_reg_32 = ADE9000_SPI_Read_32(ADDR_MASK1);

	value_reg_16 = 0x0001;
	ADE9000_SPI_Write_16(ADDR_RUN, value_reg_16);
}
void Start_Waveform_Buffer() {
	uint16_t value_reg_16;
	//WFB_CFG
	//Res,Res,Res,IN
	//Res,Res,Source
	//Mode,Fixed/resample,Avvio
	//channel burst
	value_reg_16 = 0x0038; //no IN, sinc4, stop full, fixed rate, start, solo IA
	ADE9000_SPI_Write_16(ADDR_WFB_CFG ,value_reg_16);

}

void test_read_write_reg(){
	uint16_t data_16;
	uint32_t data_32;

	printf("Attesa 2 sec\r\n");
	HAL_Delay(2000);
	//Lettura reg RUN (prima)
	data_16 = ADE9000_SPI_Read_16(ADDR_RUN);
	printf("ADDR_RUN = %x \r\n", data_16);

	//Scrittura reg RUN
	data_16 = 0x0001;
	ADE9000_SPI_Write_16(ADDR_RUN, data_16);
	printf("ADDR_RUN scritto = %x \r\n",  data_16);

	//Lettura reg RUN (dopo)
	data_16 = ADE9000_SPI_Read_16(ADDR_RUN);
	printf("ADDR_RUN = %x \r\n",  data_16);

	//Lettura reg SPI
	data_16 = ADE9000_SPI_Read_16(ADDR_LAST_CMD);
	printf("ADDR_LAST_CMD = %x \r\n",  data_16);
	data_16 = ADE9000_SPI_Read_16(ADDR_LAST_DATA_16);
	printf("ADDR_LAST_DATA_16 = %x \r\n",  data_16);

	//Lettura e scrittura 32
	data_32 = ADE9000_SPI_Read_32(ADDR_VLEVEL);
	printf("ADDR_VLEVEL = %x \r\n",  data_32);
	data_32 = 0x0022EA28;
	ADE9000_SPI_Write_32(ADDR_VLEVEL,data_32);
	printf("ADDR_VLEVEL scritto = %x \r\n",  data_32);
	data_32 = ADE9000_SPI_Read_32(ADDR_VLEVEL);
	printf("ADDR_VLEVEL = %x \r\n",  data_32);


}
