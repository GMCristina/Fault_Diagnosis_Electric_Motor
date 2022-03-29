/*
 * FFT.c
 *
 *  Created on: 25 mar 2022
 *      Author: M.Cristina Giannini
 */
#include<FFT.h>

//float complex vec[N_SAMPLE];
//float complex app[N_SAMPLE];


int log2_c(int N)
{
  int k = N, i = 0;
  while(k) {
    k >>= 1;
    i++;
  }
  return i - 1;
}

int reverse(int N, int n)
{
  uint32_t step = log2_c(N);
  int j, p = 0;
  for(j = 1; j <= step; j++) {
    if(n & (1 << (step - j)))
      p |= 1 << (j - 1);
  }
  return p;
}

void order(float complex* f1, int N)
{
  float complex *app = (float complex*)malloc(N_SAMPLE * sizeof(float complex));

  for(int i = 0; i < N; i++)
    app[i] = f1[reverse(N, i)];
  for(int j = 0; j < N; j++)
    f1[j] = app[j];
  free(app);
  app=NULL;
}

void orderInplace (float complex* f, int N){
    int j;
    float complex app;

    for(int i = 0; i < N; i++){
        j=reverse(N, i);
        if(i<j){
            app=f[i];
            f[i]=f[j];
            f[j]=app;
        }
    }
}

void FFT(float complex* f, int N){
	//order(f, N);
	orderInplace(f,N);
	uint32_t step = log2_c(N);
/*
	float complex *W = (float complex*)malloc(N_SAMPLE/2 * sizeof(float complex));
	      W[0] = 1;
	      for(int i = 1; i < N / 2; i++)
	        W[i] = cos(-2*M_PI*i/N)+I*sin(-2*M_PI*i/N);
*/
	      int n = 1;
	      int a = N / 2;
	      for(int j = 0; j < step; j++) {
	        for(int i = 0; i < N; i++) {
	          if(!(i & n)) {
	            float complex temp = f[i];
	            uint32_t idx = (i * a) % (n * a);
	            float complex W = cos(-2*M_PI*idx/N)+I*sin(-2*M_PI*idx/N);
	            //float complex Temp = W[(i * a) % (n * a)] * f[i + n];
	            float complex Temp = W * f[i + n];
	            f[i] = temp + Temp;
	            f[i + n] = temp - Temp;

	          }
	        }
	        n *= 2;
	        a = a / 2;
	      }
}

void IFFT(float complex* f, int N){
	FFT(f,N);
	f[0]=f[0]/N;
	for(uint32_t i=1;i<=N/2;i++){
		float complex app;
		app = f[i];
		f[i] = f[N-i]/N;
		f[N-i]= app/N;
	}

}
