
#define _USE_MATH_DEFINES // for C++ 
#include "mex.h"
#include <omp.h>
#include <cmath>
#include <time.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double *F_ptr, *pc_ptr;
	double *ind_ptr, *co_ind_ptr;

	srand(time(NULL));
	F_ptr=mxGetPr(prhs[0]);
	mwSize P=mxGetM(prhs[0]);
	//pc_ptr=mxGetPr(prhs[1]);
	
	plhs[0]=mxCreateDoubleMatrix(P, 1, mxREAL);
	ind_ptr=mxGetPr(plhs[0]);
	//plhs[1]=mxCreateDoubleMatrix(P, 1, mxREAL);
	//co_ind_ptr=mxGetPr(plhs[1]);

	int k;
	#pragma omp parallel for num_threads(4)
	for (k=0;k<P;k++)
	{
		int a=rand() % P;
		int b=rand() % P;
		while (b==a)
			b=rand() % P;
		int c=rand() % P;
		while (c==a || c==b)
			c=rand() % P;
		double fa=*(F_ptr+a);
		double fb=*(F_ptr+b);
		double fc=*(F_ptr+c);
		int m=a; double fm=fa;
		(void)((fm<fb) && (fm=fb, m=b));
		(void)((fm<fc) && (fm=fc, m=c));
		*(ind_ptr+k)=(m+1);
		//*(co_ind_ptr+k)=(((double)rand()/RAND_MAX)<*pc_ptr)?1:0;
	}
}