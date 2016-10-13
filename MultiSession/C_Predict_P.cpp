// mex file to calculate divergence
// input:   mxn smoothed gray image
// output:  mxn matrix, value at (i,j) refers to divergence at that pixel
#include "mex.h"
#include <omp.h>
#include <math.h>


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double *P_ptr, *t_ptr, *phi_ptr, *v_ptr,*Predict_P_ptr;
	mwSize P_no, t_no;
	int k;
	P_ptr=mxGetPr(prhs[0]);
	t_ptr=mxGetPr(prhs[1]);
	phi_ptr=mxGetPr(prhs[2]);
	v_ptr=mxGetPr(prhs[3]);
	P_no=mxGetN(prhs[0]);
	t_no=mxGetM(prhs[1]);
	
	plhs[0]=mxCreateDoubleMatrix(8, P_no, mxREAL);
	Predict_P_ptr=mxGetPr(plhs[0]);
	#pragma omp parallel for num_threads(4)
    for (k=0;k<P_no;k++)
	{
		double x=*(P_ptr+8*k);
		double z=*(P_ptr+8*k+1);
		double theta=*(P_ptr+8*k+2);
		double t,v,phi,dx,dz,dtheta;
		for (int j=0; j<t_no; j++)
		{
			t=*(t_ptr+j);
			v=*(v_ptr+j);
			phi=*(phi_ptr+j);
			dx=cos(theta)*v;
			dz=sin(theta)*v;
			dtheta=-tan(phi)*v/1.28;
			x=x+dx*t;
			z=z+dz*t;
			theta=theta+dtheta*t;
		}
		*(Predict_P_ptr+8*k)=x;
		*(Predict_P_ptr+8*k+1)=z;
		*(Predict_P_ptr+8*k+2)=theta;
		*(Predict_P_ptr+8*k+3)=dx;
		*(Predict_P_ptr+8*k+4)=dz;
		*(Predict_P_ptr+8*k+5)=dtheta;
		*(Predict_P_ptr+8*k+6)=v;
		*(Predict_P_ptr+8*k+7)=phi;
	}
}

/*
	double *ImgPtr, *divPtr;
    mwSize mrows;
    mwSize ncols;
    int k;
	mxArray *U,*V;
    double *UPtr, *VPtr;
    ImgPtr=mxGetPr(prhs[0]);
    mrows=mxGetM(prhs[0]);
    ncols=mxGetN(prhs[0]);
    plhs[0]=mxCreateDoubleMatrix(mrows, ncols, mxREAL);
	divPtr=mxGetPr(plhs[0]);

	U=mxCreateDoubleMatrix(mrows, ncols, mxREAL);
    UPtr=mxGetPr(U);
	V=mxCreateDoubleMatrix(mrows, ncols, mxREAL);
    VPtr=mxGetPr(V);

	#pragma omp parallel for num_threads(4)
    for (k=mrows+1;k<(ncols*mrows-mrows-1) ;k++)
    {
            double Grad_x=(*(ImgPtr+k+mrows)-*(ImgPtr+k-mrows));
			double Grad_y=(*(ImgPtr+k+1)-*(ImgPtr+k-1));
			double ssd11=Grad_x*Grad_x;
			double ssd12=Grad_x*Grad_y;
			double ssd22=Grad_y*Grad_y;
			double Ssdi12=ssd12*.6306+(ssd11+ssd22)*0.0838;
			double Ssdi11_22=(ssd11-ssd22)*.608;
			double EigenValue_22D12=0.5*(Ssdi11_22+sqrt(Ssdi11_22*Ssdi11_22+4*Ssdi12*Ssdi12))/Ssdi12;
			if (Ssdi12==0)
				EigenValue_22D12=0;
			double wpsdiV=sqrt(1/(1+EigenValue_22D12*EigenValue_22D12));
			double wpsdiU=wpsdiV*EigenValue_22D12;
			double psdi=wpsdiU*Grad_x+wpsdiV*Grad_y;
			if (psdi>0) {
				*(UPtr+k)=wpsdiU;
				*(VPtr+k)=wpsdiV;
			}
			else if (psdi==0) {
				*(UPtr+k)=0;
				*(VPtr+k)=0;
			}
			else {
				*(UPtr+k)=-wpsdiU;
				*(VPtr+k)=-wpsdiV;
			}
    }

	#pragma omp parallel for num_threads(4)
    for (k=mrows+1;k<(ncols*mrows-mrows-1) ;k++)
	{
		*(divPtr+k)=-0.5*(*(UPtr+k+mrows)-*(UPtr+k-mrows)+*(VPtr+k+1)-*(VPtr+k-1));
	}
	mxDestroyArray(U);
	mxDestroyArray(V);
}*/