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

float Ea;
float Ed [N_LEVEL_WAVELET];


void FD_Wavedec_zpd(float* dec, uint16_t* dec_dim, float* y){
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


void FD_Wavedec_sym(float* dec, uint16_t* dec_dim, float* y){
	uint16_t dim_y = N_SAMPLE;
	uint16_t dim_conv = dim_y + DIM_FILTER_WAVELET - 1;
	uint16_t dim_coeff = (int)dim_conv/2;

	uint16_t index_border;

	uint16_t index_dec = 0;

	for(int16_t k =0;k<N_DEC_WAVELET;k++){
		dec[k]=0;
	}

	for(uint16_t k=0; k<N_LEVEL_WAVELET; k++){
		//DIM
		Wavelet_dec_dim[k]=dim_coeff;

		//DETT
		for (uint16_t i = 0; i < dim_conv; i++) {
			for (uint16_t j = 0; j < DIM_FILTER_WAVELET; j++) {
				if ((i % 2) == 1) { //downsampling (solo pari matlab(da 1)=solo dispari in C (da 0))
					if((i-j)<0){ //gestire primi elementi
						index_border = -(i-j)-1;
						dec[index_dec+i/2] = dec[index_dec+i/2] + y[index_border] * HiD[j];
					} else if((i-j)>=dim_y){ //gestire ultimi elementi
						index_border = (dim_y-1)-((i-j)-dim_y);
						dec[index_dec+i/2] = dec[index_dec+i/2] + y[index_border] * HiD[j];
					} else {
						dec [index_dec+i/2] = dec[index_dec+i/2] + y[i-j] * HiD[j];
					}
				}
			}
		}
		//APPR
		index_dec = index_dec + dim_coeff;
		for (uint16_t i = 0; i < dim_conv; i++) {
			for (uint16_t j = 0; j < DIM_FILTER_WAVELET; j++) {
				if ((i % 2) == 1) { //downsampling (solo pari matlab(da 1)=solo dispari in C (da 0))
					if((i-j)<0){ //gestire primi elementi
						index_border = -(i-j)-1;
						dec[index_dec+i/2] = dec[index_dec+i/2] + y[index_border] * LoD[j];
					} else if((i-j)>=dim_y){ //gestire ultimi elementi
						index_border = (dim_y-1)-((i-j)-dim_y);
						dec[index_dec+i/2] = dec[index_dec+i/2] + y[index_border] * LoD[j];
					} else {
						dec [index_dec+i/2] = dec[index_dec+i/2] + y[i-j] * LoD[j];
					}
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

void FD_Wenergy(float* dec, uint16_t* dec_dim, float* Ea, float* Ed){
	float tot=0;
	uint16_t index=0;
	uint16_t dim = 0;

	*Ea =0;
	for(uint16_t i=0;i<N_LEVEL_WAVELET;i++){
		Ed[i]=0;
	}

	for(uint16_t i=0;i<N_LEVEL_WAVELET;i++){
		dim = dec_dim[i];
		for(uint16_t j=0;j<dim;j++){
			Ed[i]=Ed[i]+power(dec[index+j],2);
		}
		index = index + dim;
	}

	dim = dec_dim[N_LEVEL_WAVELET-1];
	for(uint16_t j=0;j<dim;j++){
		*Ea=*Ea+power(dec[index+j],2);
	}

	tot = *Ea;
	for(uint16_t i=0;i<N_LEVEL_WAVELET;i++){
		tot = tot + Ed[i];
	}

	*Ea = 100*(*Ea)/tot;
	for(uint16_t i=0;i<N_LEVEL_WAVELET;i++){
		Ed[i] = 100*Ed[i]/tot;
	}
}

float power(float a, uint16_t b){
	if(b == 0)
		return 1;
	else
		return a *= power(a,b-1);
}
