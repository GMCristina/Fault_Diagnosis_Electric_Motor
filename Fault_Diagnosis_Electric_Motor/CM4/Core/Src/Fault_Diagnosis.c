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


