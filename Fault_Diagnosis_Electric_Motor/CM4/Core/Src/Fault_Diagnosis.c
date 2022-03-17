/*
 * Fault_Diagnosis.c
 *
 *  Created on: 16 mar 2022
 *      Author: M.Cristina Giannini
 */

#include<Fault_Diagnosis.h>
//Filter for wavelet db5
float LoD [DIM_FILTER_WAVELET] = {0.0033,-0.0126,-0.0062,0.0776,-0.0322,-0.2423,0.1384,0.7243,0.6038,0.1601};
float HiD [DIM_FILTER_WAVELET] = {-0.1601,0.6038,-0.7243,0.1384,0.2423,-0.0322,-0.0776,-0.0062,0.0126,0.0033};

float Wavelet_dec[N_DEC_WAVELET];
uint16_t Wavelet_dec_dim[N_LEVEL_WAVELET];

void FD_Wavedec(float* dec, uint16_t* dec_dim, float* y){
	uint16_t dim_y = N_SAMPLE;
	uint16_t dim_conv = dim_y + DIM_FILTER_WAVELET - 1;
	uint16_t dim_coeff = (int)dim_conv/2;

	uint16_t index_dec = 0;

	for(int16_t k =0;k<N_DEC_WAVELET;k++){
		dec[k]=0;
	}

	for(uint16_t k=0; k<N_LEVEL_WAVELET; k++){
		//DIM
		Wavelet_dec_dim[k]=dim_coeff;

		//DETT
		for (uint16_t i = 0; i < dim_y; i++) {
			for (uint16_t j = 0; j < DIM_FILTER_WAVELET; j++) {
				if (((i + j) % 2) == 1) { //downsampling (solo pari matlab(da 1)=solo dispari in C (da 0))
					dec [index_dec+(i + j)/2] = dec[index_dec+(i + j)/2] + y[i] * HiD[j];
				} else if(((i + j) % 2) == 0){
					uint16_t temp = i+j;
				}
			}
		}
		//APPR
		index_dec = index_dec + dim_coeff;
		for (uint16_t i = 0; i < dim_y; i++) {
			for (uint16_t j = 0; j < DIM_FILTER_WAVELET; j++) {
				if (((i + j) % 2) == 1) { //downsampling (solo pari)
					dec[index_dec + (i + j)/2] = dec[index_dec + (i + j)/2] +y[i] * LoD[j];
				}
			}
		}

		if(!(k==N_LEVEL_WAVELET-1)){
		dim_y = dim_coeff;

		for(uint16_t i=0; i< dim_y;i++){
			y[i] = dec[index_dec + i];
			dec[index_dec+i]=0;
		}

		dim_conv = dim_y + DIM_FILTER_WAVELET - 1;
		dim_coeff = (int)dim_conv/2;
		}
	}
}
