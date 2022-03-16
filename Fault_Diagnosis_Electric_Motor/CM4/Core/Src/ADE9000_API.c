/*
 * ADE9000_API.c
 *
 *  Created on: Feb 14, 2022
 *      Author: M.Cristina Giannini
 */

#include "ADE9000_API.h"
#include "main.h"

int8_t flag_read = 0;
int32_t n_int = 0;

union DATA va[N_SAMPLE], ia[N_SAMPLE];

void ADE9000_Setup(){
	uint32_t value_reg_32;
	uint16_t value_reg_16;

	// ADDR_PGA_GAIN
	value_reg_16 = 0x0000; //gain all channel 1
	ADE9000_SPI_Write_16(ADDR_PGA_GAIN,value_reg_16);

	//CONFIG2
	value_reg_16 = 	0x0C00;			//Default High pass corner frequency of 1.25Hz
	ADE9000_SPI_Write_16(ADDR_CONFIG2,value_reg_16);

	//CONFIG1
	//EXT_REF off
	value_reg_16 = 0x000000;
	ADE9000_SPI_Write_16(ADDR_CONFIG1,value_reg_16);

	//ACCMODE
	value_reg_16= 0x0000;			//3P4W Wye configuration
	ADE9000_SPI_Write_16(ADDR_ACCMODE,value_reg_16);

	//WFB_CFG
	//value_reg_16 = 0x0028; //no IN, sinc4, stop full, fixed rate, stop, solo IA (1000)
	//value_reg_16 = 0x1020;//IN, sinc4, stop full, fixed rate, stop, tutti canali(0000)
	//value_reg_16 = 0x0029; //no IN, sinc4, stop full, fixed rate, stop, solo VA (1001)
	//value_reg_16 = 0x0021; //no IN, sinc4, stop full, fixed rate, stop, solo Ia e VA (0001)
	//value_reg_16 = 0x0221; //no IN, LPF, stop full, fixed rate, stop, solo Ia e VA (0001)

	value_reg_16 = 0x0000;
	value_reg_16 = value_reg_16 | (WF_IN_EN<<12);
	value_reg_16 = value_reg_16 | (WF_SRC<<8);
	value_reg_16 = value_reg_16 | (WF_MODE<<6);
	value_reg_16 = value_reg_16 | (WF_CAP_SEL<<5);
	value_reg_16 = value_reg_16 | BURST_CHAN;

	ADE9000_SPI_Write_16(ADDR_WFB_CFG ,value_reg_16);

	//WFB_PG_IRQEN
	// 1 bit per pagina: pag15-pag0
	//fa PageFULL in STATUS0

	//value_reg_16 = 0x8000; //page 15
	//value_reg_16 = 0xFFFF; //all page
	value_reg_16 = 0x8080; //page 15, page 7 (metà)
	//value_reg_16 = 0x0000; //no int
	ADE9000_SPI_Write_16(ADDR_WFB_PG_IRQEN,value_reg_16);

	//ADDR_MASK0
	//IRQ0 per full page (bit 17)
	//quando una delle pagine settate è piena
	value_reg_32 = 0x00020000;
	ADE9000_SPI_Write_32(ADDR_MASK0,value_reg_32);
	value_reg_32 = ADE9000_SPI_Read_32(ADDR_MASK0);

	//ADDR_MASK1
	//disable all int
	value_reg_32 = 0x00000000;
	ADE9000_SPI_Write_32(ADDR_MASK1,value_reg_32);
	value_reg_32 = ADE9000_SPI_Read_32(ADDR_MASK1);

	//ADDR_RUN
	//Start ADE9000 measurement
	value_reg_16 = 0x0001;
	ADE9000_SPI_Write_16(ADDR_RUN, value_reg_16);
}

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

    /*
    //Check end power-up sequence (RESTDONE)
    uint32_t value_reg_32;
    do {
    	value_reg_32 = ADE9000_SPI_Read_32(ADDR_STATUS1);
    	HAL_Delay(500);
    } while ((value_reg_32 & 0x00010000)==0);
    //Clear IRQ1
    value_reg_32 = value_reg_32 & 0x00010000;
    ADE9000_SPI_Write_32(ADDR_STATUS1,value_reg_32);
    */
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
	ret = HAL_SPI_Transmit(&hspi1,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//Receive data
	ret = HAL_SPI_Receive(&hspi1,data.data_8,SIZE_16,TIMEOUT_SPI);

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
	ret = HAL_SPI_Transmit(&hspi1,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//Receive data
	ret = HAL_SPI_Receive(&hspi1,data.data_8 + 2,SIZE_16,TIMEOUT_SPI);
	ret = HAL_SPI_Receive(&hspi1,data.data_8,SIZE_16,TIMEOUT_SPI);

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
	ret = HAL_SPI_Transmit(&hspi1,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//Send data
	data.data_16 = Data;
	ret = HAL_SPI_Transmit(&hspi1,data.data_8,SIZE_16,TIMEOUT_SPI);

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
	ret = HAL_SPI_Transmit(&hspi1,addr.data_8,SIZE_16,TIMEOUT_SPI);

	//Send data
	data.data_32 = Data;
	ret = HAL_SPI_Transmit(&hspi1,data.data_8 +2,SIZE_16,TIMEOUT_SPI);
	ret = HAL_SPI_Transmit(&hspi1,data.data_8,SIZE_16,TIMEOUT_SPI);

	//CS off
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);

}


void Start_Waveform_Buffer() {
	uint16_t value_reg_16;
	value_reg_16 = ADE9000_SPI_Read_16(ADDR_WFB_CFG);
	value_reg_16 = (value_reg_16|0x0010);
	ADE9000_SPI_Write_16(ADDR_WFB_CFG ,value_reg_16);
}

void Stop_Waveform_Buffer(){
	uint16_t value_reg_16;
	value_reg_16 = ADE9000_SPI_Read_16(ADDR_WFB_CFG);
	value_reg_16 = (value_reg_16 & 0xFFEF);
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

	data_32 = 0x00020000;
	ADE9000_SPI_Write_32(ADDR_MASK0,data_32);
	data_32 = ADE9000_SPI_Read_32(ADDR_MASK0);

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

void ADE9000_SPI_Burst_Read_one_ch(uint16_t Address, uint16_t n, int32_t* data){
	union ADE_DATA_32 app;
	union ADE_DATA_16 addr;
	HAL_StatusTypeDef ret;

	//Address for read
	// addr|R/W(1/0)|000
	addr.data_16 = (((Address <<4) & 0xFFF0)+8);

	//CS on
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Send address
	ret = HAL_SPI_Transmit(&hspi1,addr.data_8,SIZE_16,TIMEOUT_SPI);

	for(uint16_t i=0; i<n; i++){
		//Receive data
		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(data + i)= app.data_32;
	}

	//CS off
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
}

void ADE9000_SPI_Burst_Read_two_ch(uint16_t Address, uint16_t n, int32_t* i, int32_t* v){
	union ADE_DATA_32 app;
	union ADE_DATA_16 addr;
	HAL_StatusTypeDef ret;

	//Address for read
	// addr|R/W(1/0)|000
	addr.data_16 = (((Address <<4) & 0xFFF0)+8);

	//CS on
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Send address
	ret = HAL_SPI_Transmit(&hspi1,addr.data_8,SIZE_16,TIMEOUT_SPI);

	for(uint16_t j=0; j<n; j++){
		//Receive data

		//NB: CONTROLLARE ORDINE (REGISTRI è I,V)
		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(i + j)= app.data_32;

		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(v + j)= app.data_32;
	}

	//CS off
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
}

void ADE9000_SPI_Burst_Read_all(uint16_t Address, uint16_t n, int32_t* ia, int32_t* ib, int32_t* ic, int32_t* in, int32_t* va, int32_t* vb, int32_t* vc){
	union ADE_DATA_32 app;
	union ADE_DATA_16 addr;
	HAL_StatusTypeDef ret;

	//Address for read
	// addr|R/W(1/0)|000
	addr.data_16 = (((Address <<4) & 0xFFF0)+8);

	//CS on
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

	//Send address
	ret = HAL_SPI_Transmit(&hspi1,addr.data_8,SIZE_16,TIMEOUT_SPI);

	for(uint16_t i=0; i<n; i++){
		//Receive data
		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(ia + i)= app.data_32;
		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(va + i)= app.data_32;

		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(ib + i)= app.data_32;
		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(vb + i)= app.data_32;

		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(ic + i)= app.data_32;
		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(vc + i)= app.data_32;

		ret = HAL_SPI_Receive(&hspi1,app.data_8 + 2,SIZE_16,TIMEOUT_SPI);
		ret = HAL_SPI_Receive(&hspi1,app.data_8,SIZE_16,TIMEOUT_SPI);
		*(in + i)= app.data_32;

	}

	//CS off
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
}

void ADE9000_Conv_32_24(int32_t* data, uint32_t n){
	int32_t app, err = 0;
	for(uint32_t i=0; i<n; i++){
		app = *(data + i);
		if((app & 0x0000000F)!=0) {
			err +=1;
		}
		if ((app &0xF0000000)==0xF0000000){
			app = ((app>>4)|0xF0000000);
		}else if((app &0xF0000000)==0x00000000) {
			app = ((app>>4)|0x00000000);
		}
		else {
			err +=1;
		}
		*(data+i) = app;
	}
	printf("errori: %d\r\n",err);

}

void ADE9000_Conv_ADC_I(union DATA *data_i, uint32_t n) {
	if (ACQUISITION_FREQ == 32000) {
		for (uint32_t i = 0; i < n; i++) {
			data_i[i].data_float = ((float) data_i[i].data_int * V_REF / FULL_SCALE_CODE_SINC4) / FDT_I;
		}
	} else if (ACQUISITION_FREQ == 8000) {
		for (uint32_t i = 0; i < n; i++) {
			data_i[i].data_float = ((float) data_i[i].data_int * V_REF / FULL_SCALE_CODE_LPF) / FDT_I;
		}
	}

	for (uint32_t i = 0; i < n; i++) {
				data_i[i].data_float = (data_i[i].data_float - OFFSET_I)*GAIN_I;
	}


}

void ADE9000_Conv_ADC_V(union DATA *data_v, uint32_t n) {
	if (ACQUISITION_FREQ == 32000) {
		for (uint32_t i = 0; i < n; i++) {
			data_v[i].data_float = ((float) data_v[i].data_int * V_REF / FULL_SCALE_CODE_SINC4) / FDT_V;
		}
	} else if (ACQUISITION_FREQ == 8000) {
		for (uint32_t i = 0; i < n; i++) {
			data_v[i].data_float = ((float) data_v[i].data_int * V_REF / FULL_SCALE_CODE_LPF) / FDT_V;
		}
	}

	for (uint32_t i = 0; i < n; i++) {
				data_v[i].data_float = (data_v[i].data_float - OFFSET_V)*GAIN_V;
	}

}
