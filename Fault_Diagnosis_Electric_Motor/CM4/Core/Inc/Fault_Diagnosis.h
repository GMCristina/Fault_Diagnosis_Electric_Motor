/*
 * Fault_Diagnosis.h
 *
 *  Created on: 16 mar 2022
 *      Author: M.Cristina Giannini
 */

#ifndef INC_FAULT_DIAGNOSIS_H_
#define INC_FAULT_DIAGNOSIS_H_

#include <stdlib.h>

#include<ADE9000_API.h>
#include<FFT.h>

//Wavelet parameters
//wavelet db5 -> 10 element filter
#define DIM_FILTER_WAVELET 10
extern float LoD [DIM_FILTER_WAVELET];
extern float HiD [DIM_FILTER_WAVELET];

#define N_LEVEL_WAVELET 10

#define L1 (int32_t)((N_SAMPLE + N_LEVEL_WAVELET - 1)/2)
#define L2 (int32_t)((L1 + N_LEVEL_WAVELET - 1)/2)
#define L3 (int32_t)((L2 + N_LEVEL_WAVELET - 1)/2)
#define L4 (int32_t)((L3 + N_LEVEL_WAVELET - 1)/2)
#define L5 (int32_t)((L4 + N_LEVEL_WAVELET - 1)/2)
#define L6 (int32_t)((L5 + N_LEVEL_WAVELET - 1)/2)
#define L7 (int32_t)((L6 + N_LEVEL_WAVELET - 1)/2)
#define L8 (int32_t)((L7 + N_LEVEL_WAVELET - 1)/2)
#define L9 (int32_t)((L8 + N_LEVEL_WAVELET - 1)/2)
#define L10 (int32_t)((L9 + N_LEVEL_WAVELET - 1)/2)

#define N_DEC_WAVELET L1+L2+L3+L4+L5+L6+L7+L8+L9+L10+L10

//External variables
extern uint16_t Wavelet_dec_dim[N_LEVEL_WAVELET];

extern union DATA ia[N_SAMPLE+N_DEC_WAVELET];

extern float Ea;
extern float Ed [N_LEVEL_WAVELET];

//Functions declaration
void FD_Wavedec_zpd(float* dec, uint16_t* dec_dim,float* y);
void FD_Wavedec_sym(float* dec, uint16_t* dec_dim,float* y);

void FD_Wenergy(float* dec, uint16_t* dec_dim, float* Ea, float* Ed);

void FD_Hilbert(float*y);
void FD_Hilbert_fast(float*y);

#endif /* INC_FAULT_DIAGNOSIS_H_ */
