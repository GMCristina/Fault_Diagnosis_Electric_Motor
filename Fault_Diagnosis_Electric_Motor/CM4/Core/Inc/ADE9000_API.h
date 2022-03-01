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

#define ACQUISITION_PERIOD 1 //seconds

#define TIMEOUT_SPI 100
//SPI with 16 bit data frame
#define SIZE_16 1
#define BURST_READ_N 256
#define WAVEFORM_BUFFER_START_ADDR 0x800

//Waveform buffer config
//0 disable, 1 enable IN
#define WF_IN_EN 0b1

//00(sinc4),10(sinc4+lpf),11(pcf)
#define WF_SRC 0b10

//00(stop full), 01(stop trigger)
//10(stop trigger center),11(save add trigger)
#define WF_MODE 0b00

//0(resampled),1(fixed rate)
#define WF_CAP_SEL 0b1

// 0000(all)
// 0001 (ia,va), 0010(ib,vb), 0011(ic,vc)
// 1000(ia), 1001(va)
// 1010(ib), 1011(vb)
// 1100(ic), 1101(vc)
// 1110(in), 1111(single add)
#define BURST_CHAN 0b0000

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
