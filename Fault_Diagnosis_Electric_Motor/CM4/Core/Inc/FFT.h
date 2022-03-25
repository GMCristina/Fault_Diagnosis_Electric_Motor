/*
 * FFT.h
 *
 *  Created on: 25 mar 2022
 *      Author: M.Cristina Giannini
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_

#include <complex.h>
#include<math.h>

#include<Fault_Diagnosis.h>
#include<ADE9000_API.h>

#include <stdlib.h>

//extern float complex vec[N_SAMPLE]; //float complex (8byte=4*2)

int log2_c(int N);
int reverse(int N, int n);
void order( float complex* f1, int N);
void FFT(float complex* f, int N);
void IFFT(float complex* f, int N);




#endif /* INC_FFT_H_ */
