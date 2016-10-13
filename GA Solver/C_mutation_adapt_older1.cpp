
#define _USE_MATH_DEFINES // for C++ 
#include "mex.h"
#include <omp.h>
#include <cmath>
#include <time.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double *d_v_ptr, *d_phi_ptr, *pm_adapt_ptr;
	double *max_d_v_ptr, *max_d_phi_ptr;
	int k;
	srand(time(NULL));
	d_v_ptr=mxGetPr(prhs[0]);
	d_phi_ptr=mxGetPr(prhs[1]);
	pm_adapt_ptr=mxGetPr(prhs[2]);
	max_d_v_ptr=mxGetPr(prhs[3]);
	max_d_phi_ptr=mxGetPr(prhs[4]);
	mwSize P=mxGetM(prhs[0]);
	mwSize horizon=mxGetN(prhs[0]);
	#pragma omp parallel for num_threads(4)
	for (k=0;k<P;k++)
	{
		for (int j=0; j<horizon; j++)
		{
			double rd1=(double)rand()/RAND_MAX;
			if (rd1<*(pm_adapt_ptr+k))
				*(d_v_ptr+j*P+k)=2*(*max_d_v_ptr)*((double)rand()/RAND_MAX)-(*max_d_v_ptr);
			rd1=(double)rand()/RAND_MAX;
			if (rd1<*(pm_adapt_ptr+k))
			{
				double sign_d_phi=(*(d_phi_ptr+j*P+k)>0?1:-1);
				*(d_phi_ptr+j*P+k)=*max_d_phi_ptr*((double)rand()/RAND_MAX)*sign_d_phi;
			}
		}
	}
}