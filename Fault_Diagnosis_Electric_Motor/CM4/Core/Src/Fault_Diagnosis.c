/*
 * Fault_Diagnosis.c
 *
 *  Created on: 16 mar 2022
 *      Author: M.Cristina Giannini
 */


#include<Fault_Diagnosis.h>
#include<math.h>

#define ARM_MATH_CM4
#include <arm_math.h>

//Filter for wavelet db5
float LoD [DIM_FILTER_WAVELET] = {0.0033,-0.0126,-0.0062,0.0776,-0.0322,-0.2423,0.1384,0.7243,0.6038,0.1601};
float HiD [DIM_FILTER_WAVELET] = {-0.1601,0.6038,-0.7243,0.1384,0.2423,-0.0322,-0.0776,-0.0062,0.0126,0.0033};

//float Wavelet_dec[N_DEC_WAVELET];
uint16_t Wavelet_dec_dim[N_LEVEL_WAVELET];

//float FFT_r[N_SAMPLE/2 +1];
//float FFT_i[N_SAMPLE/2 +1];
//float y_1[N_SAMPLE] = {0.957506835434298,0.964888535199277,0.157613081677548,0.970592781760616,0.957166948242946,0.485375648722841,0.800280468888800,0.141886338627215,0.421761282626275,0.915735525189067,0.792207329559554,0.959492426392903,0.655740699156587,0.0357116785741896,0.849129305868777,0.933993247757551};


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
			Ed[i]=Ed[i]+pow(dec[index+j],2);
		}
		index = index + dim;
	}

	dim = dec_dim[N_LEVEL_WAVELET-1];
	for(uint16_t j=0;j<dim;j++){
		*Ea=*Ea+pow(dec[index+j],2);
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


/*
void FD_Hilbert(float* y){

	float* FFT_r = &Wavelet_dec[0];
	float* FFT_i = &Wavelet_dec[N_SAMPLE/2+1];


	float arg, sine, cosine, fat1, fat2;

	fat1 = 2*PI/N_SAMPLE;
	//FFT
	for(uint32_t n=0;n<=N_SAMPLE/2;n++){
		fat2 = fat1*n;
		for(uint32_t h=0;h<N_SAMPLE;h++){
			arg = fat2*h;
			cosine = arm_cos_f32(arg);
			sine = arm_sin_f32(arg);
		       *(FFT_r + n) += y[h]*cosine;//cos(arg);
		       *(FFT_i + n) += -y[h]*sine;//sin(arg);
		}
	}
	//DELETE NEGATIVE
	for(uint32_t n=0;n<=N_SAMPLE/2;n++){
		if (n>0 && n<N_SAMPLE/2){
			*(FFT_r + n) = 2*(*(FFT_r + n));
			*(FFT_i + n) = 2*(*(FFT_i + n));
		}
	}
	//IFFT and envelope
	for(uint32_t n=0;n<N_SAMPLE;n++){
		fat2 = fat1*n;
		float app_i = 0;
			for(uint32_t h=0;h<=N_SAMPLE/2;h++){
				arg = fat2*h;
				sine = arm_sin_f32(arg);
				cosine = arm_cos_f32(arg);
			      app_i += (*(FFT_r + h))*sine + (*(FFT_i + h))*cosine;
			}
			app_i=app_i/N_SAMPLE;
			y[n] = sqrt(app_i*app_i + y[n]*y[n]);
	}


}
*/

void FD_Hilbert_fast(float*y){
/*
	float complex *x = (float complex*)malloc(N_SAMPLE * sizeof(float complex));

	for(uint32_t n=0;n<N_SAMPLE;n++){
		x[n]=y[n];
	}

	free(y);
	y=NULL;

	*/

	y = (float*)realloc(y,2*N_SAMPLE*sizeof(float));

    int32_t j = N_SAMPLE -1;
    for(int32_t i = 2*N_SAMPLE-1;i>=0;i--){
       y[i] = 0;
       i--;
       y[i]=y[j];
       j--;
    }

    float complex* x = (float complex*)y;

	FFT(x,N_SAMPLE);

	//DELETE NEGATIVE
	for(uint32_t n=0;n<N_SAMPLE;n++){
		if (n>0 && n<N_SAMPLE/2){
			x[n] = 2*x[n];
		} else if(n>N_SAMPLE/2) {
			x[n]=0;
		}
	}

	IFFT(x,N_SAMPLE);

	y = (float *)malloc(N_SAMPLE * sizeof(float));


	for(uint32_t n=0;n<N_SAMPLE;n++){
		float hil = cimagf(x[n]);
		float signal = creal(x[n]);//y[n];
		y[n] = sqrt(hil*hil + signal*signal);
	}
	 free(x);
	 x = NULL;
}
