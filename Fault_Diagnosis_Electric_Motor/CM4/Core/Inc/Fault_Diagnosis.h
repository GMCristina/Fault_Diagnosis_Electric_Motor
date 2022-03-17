/*
 * Fault_Diagnosis.h
 *
 *  Created on: 16 mar 2022
 *      Author: M.Cristina Giannini
 */

#ifndef INC_FAULT_DIAGNOSIS_H_
#define INC_FAULT_DIAGNOSIS_H_

#include<ADE9000_API.h>

//wavelet db5 -> 10 element filter
#define DIM_FILTER_WAVELET 10

extern float LoD [DIM_FILTER_WAVELET];
extern float HiD [DIM_FILTER_WAVELET];

#define N_LEVEL_WAVELET 10
#define N_DEC_WAVELET 32083 //dim c da Matlab

extern float Wavelet_dec[N_DEC_WAVELET];
extern uint16_t Wavelet_dec_dim[N_LEVEL_WAVELET];

void FD_Wavedec(float* dec, uint16_t* dec_dim,float* y);




#endif /* INC_FAULT_DIAGNOSIS_H_ */
