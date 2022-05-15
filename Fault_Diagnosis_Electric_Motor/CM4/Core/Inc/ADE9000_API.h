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
#include <math.h>

//Acquisition parameters
//NB: Acquire buffer and acquisition period for N_SAMPLE=2^n
#define WAVEFORM_BUFFER_DIM 256
#define ACQUISITION_PERIOD 2.048//1.024 //seconds
#define ACQUISITION_FREQ 8000//8000 //HZ
//16 pagine, 16 campioni per pagina
//32KHz 8ms tot, 0.5ms per pagina
//8KHz 32ms tot, 2ms per pagina
#define N_BUFFER (int32_t)((ACQUISITION_PERIOD*ACQUISITION_FREQ)/WAVEFORM_BUFFER_DIM)
#define N_SAMPLE (WAVEFORM_BUFFER_DIM*N_BUFFER) //n_buffer x dim_buffer(256)
#define BURST_READ_N (WAVEFORM_BUFFER_DIM/2) // metà buffer (16samp*8pagine)

//Convertion counts-A parameters
#define FULL_SCALE_CODE_SINC4 67076544
#define FULL_SCALE_CODE_LPF 74481024
#define V_REF 1

#define FDT_V (1000.0/(4*200000+1000)) // 1/801

#define N_CT 2500 //1500(rosso VAC4658_X043),2500(nero VAC4626_X002)
#define R_I (5.1 *2)
#define FDT_I (R_I/N_CT)

#define OFFSET_V 0.0//0.4 V
#define OFFSET_I 0.0//0.08 A
#define GAIN_V 1.0
#define GAIN_I 1.0

//SPI parameters
#define TIMEOUT_SPI 100
#define SIZE_16 1 //SPI with 16 bit data frame

//Waveform buffer parameters
#define WAVEFORM_BUFFER_START_ADDR 0x800

//Waveform buffer config
//0 disable, 1 enable IN
#define WF_IN_EN 0b0

//00(sinc4),10(sinc4+lpf),11(pcf)
#define WF_SRC 0b10//00

//00(stop full), 01(stop trigger)
//10(stop trigger center),11(save add trigger)
#define WF_MODE 0b01

//0(resampled),1(fixed rate)
#define WF_CAP_SEL 0b1

// 0000(all)
// 0001 (ia,va), 0010(ib,vb), 0011(ic,vc)
// 1000(ia), 1001(va)
// 1010(ib), 1011(vb)
// 1100(ic), 1101(vc)
// 1110(in), 1111(single add)
#define BURST_CHAN 0b1000

//Unions
union ADE_DATA_32{
	uint8_t data_8[4];
	uint16_t data_16[2];
	uint32_t  data_32;
};

union ADE_DATA_16{
	uint8_t data_8[2];
	uint16_t data_16;
};

union DATA{
	uint8_t data_byte[4];
	int32_t data_int;
	float data_float;
};

//External variables
extern SPI_HandleTypeDef hspi1;
extern int8_t flag_read;
extern int8_t flag_trigger;

//Functions declaration
uint16_t ADE9000_SPI_Read_16(uint16_t Address);
uint32_t ADE9000_SPI_Read_32(uint16_t Address);
void ADE9000_SPI_Write_16(uint16_t Address, uint16_t Data);
void ADE9000_SPI_Write_32(uint16_t Address, uint32_t Data);

void ADE9000_Setup(void);
void ADE9000_Power(void);

void test_read_write_reg(void);

void Start_Waveform_Buffer(void);
void Stop_Waveform_Buffer(void);

void ADE9000_SPI_Burst_Read_one_ch(uint16_t Address, uint16_t n, int32_t* data);
void ADE9000_SPI_Burst_Read_two_ch(uint16_t Address, uint16_t n, int32_t* i, int32_t* v);
void ADE9000_SPI_Burst_Read_all(uint16_t Address, uint16_t n, int32_t* ia, int32_t* ib, int32_t* ic, int32_t* in, int32_t* va, int32_t* vb, int32_t* vc);

void ADE9000_Conv_32_24(int32_t* data, uint32_t n);
void ADE9000_Conv_ADC_I(union DATA* data_i, uint32_t n);
void ADE9000_Conv_ADC_V(union DATA* data_v, uint32_t n);
void ADE9000_Remove_DC(union DATA* data_v, uint32_t n);


#endif /* INC_ADE9000_API_H_ */
