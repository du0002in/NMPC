
#define _USE_MATH_DEFINES // for C++ 
#include "mex.h"
#include <omp.h>
#include <cmath>

 static int isZero(double x)
{
return x > -0.000001 && x < 0.000001;
}
 void solveCubic(double c[4], double s[3], double *num_root)
{
int	i, num;
double	sub,
	A, B, C,
	sq_A, p, q,
	cb_p, D;

// normalize the equation:x ^ 3 + Ax ^ 2 + Bx  + C = 0
A = c[2] / c[3];
B = c[1] / c[3];
C = c[0] / c[3];

// substitute x = y - A / 3 to eliminate the quadric term: x^3 + px + q = 0

sq_A = A * A;
p = 1.0/3.0 * (-1.0/3.0 * sq_A + B);
q = 1.0/2.0 * (2.0/27.0 * A *sq_A - 1.0/3.0 * A * B + C);

// use Cardano's formula

cb_p = p * p * p;
D = q * q + cb_p;

if (isZero(D))
    {
    if (isZero(q))
	{
	// one triple solution
	s[0] = 0.0;
	num = 1;
	}
    else
	{
	// one single and one double solution
	double u = pow(-q,1.0/3.0);
	s[0] = 2.0 * u;
	s[1] = - u;
	num = 2;
	}
    }
else
    if (D < 0.0)
	{
	// casus irreductibilis: three real solutions
	double phi = 1.0/3.0 * acos(-q / sqrt(-cb_p));
	double t = 2.0 * sqrt(-p);
	s[0] = t * cos(phi);
	s[1] = -t * cos(phi + M_PI / 3.0);
	s[2] = -t * cos(phi - M_PI / 3.0);
	num = 3;
	}
    else
	{
	// one real solution
	double sqrt_D = sqrt(D);
	double u = pow(sqrt_D + fabs(q),1.0/3.0);
	if (q > 0.0)
	    s[0] = - u + p / u ;
	else
	    s[0] = u - p / u;
	num = 1;
	}

// resubstitute
sub = 1.0 / 3.0 * A;
for (i = 0; i < num; i++)
    s[i] -= sub;
*num_root=num;
}
 void solve_xc_d_theta(double *a0, double *b0, double *c0, double *x, double *z, double *theta, double *new_xc, double *new_d_theta) {
	 if (*a0<-0.0000001 || *a0>0.0000001) {	
		double co[4], s[3], num_root;
		double x0=*x,
			   z0=*z,
			   theta0=*theta;
		double a=*a0, b=*b0, c=*c0;
		co[0]=(b)*(c)-(b)*x0-z0;
		co[1]=(b)*(b)+2*(a)*(c-x0)+1;
		co[2]=3*(a)*(b);
		co[3]=2*(a)*(a);
		solveCubic(co,s,&num_root);
		if (num_root>=2)
		{
			double d_sqr_pre_inv=0;
			for (int i=0; i<num_root; i++)
			{
				double d_sqr_inv=1.0/(pow(a*s[i]*s[i]+b*s[i]+c-x0,2)+pow(s[i]-z0,2));
				if (d_sqr_inv>d_sqr_pre_inv)
				{
						s[0]=s[i];
						d_sqr_pre_inv=d_sqr_inv;
				}
			}
		}
		double z_tangent=s[0];
		double x_tangent=a*z_tangent*z_tangent+b*z_tangent+c;
		double d_theta=atan(1/(2*a*z_tangent+b));
		if (d_theta<0)
		    d_theta=d_theta+M_PI;
		*new_xc=-sin(d_theta)*(x0-x_tangent)+cos(d_theta)*(z0-z_tangent);
		*new_d_theta=(-d_theta+theta0);
	}
	else {
		double co[2], s[1];
		double x0=*x,
			   z0=*z,
			   theta0=*theta;
		double a=*a0, b=*b0, c=*c0;
		co[0]=b*c-b*x0-z0;
		co[1]=b*b+1;
		//co[2]=3*a*b;
		//co[3]=2*a*a;
		s[0]=-co[0]/co[1];
		double z_tangent=s[0];
		double x_tangent=a*z_tangent*z_tangent+b*z_tangent+c;
		double d_theta=atan(1/(2*a*z_tangent+b));
		if (d_theta<0)
		    d_theta=d_theta+M_PI;
		*new_xc=-sin(d_theta)*(x0-x_tangent)+cos(d_theta)*(z0-z_tangent);
		*new_d_theta=(-d_theta+theta0);
	}
 }

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double *d_v_ptr, *d_phi_ptr, *X0_ptr, *xc_ptr, *d_theta_ptr, *abc_ptr, *std_xc_ptr, *std_v0_ptr, *w_ptr;
	mwSize P, horizon;
	int k;
	double *F_ptr;
	double Ts=0.05;
	double deno_v=1+2.064*Ts+7.818*Ts*Ts;
	double deno_phi=1+23.15*Ts+252.3*Ts*Ts;
	double w_xc=10.0, w_d_theta=8.5, w_v=0.2, w_d_v=0.1, w_d_phi=1;

	d_v_ptr=mxGetPr(prhs[0]);
	d_phi_ptr=mxGetPr(prhs[1]);
	X0_ptr=mxGetPr(prhs[2]);
	xc_ptr=mxGetPr(prhs[3]);
	d_theta_ptr=mxGetPr(prhs[4]);
	abc_ptr=mxGetPr(prhs[5]);
	std_xc_ptr=mxGetPr(prhs[6]);
	std_v0_ptr=mxGetPr(prhs[7]);
	w_ptr=mxGetPr(prhs[8]);
	P=mxGetM(prhs[0]);
	horizon=mxGetN(prhs[1]);

	w_xc=*(w_ptr+0);
	w_d_theta=*(w_ptr+1);
	w_v=*(w_ptr+2);
	w_d_v=*(w_ptr+3);
	w_d_phi=*(w_ptr+4);
	double w_acc0=*(w_ptr+5);

	plhs[0]=mxCreateDoubleMatrix(P, 1, mxREAL);
	F_ptr=mxGetPr(plhs[0]);

	#pragma omp parallel for num_threads(4)
	for (k=0;k<P;k++)
	{
		double x=*X0_ptr;			double z=*(X0_ptr+1);		double theta=*(X0_ptr+2);
		double xdot=*(X0_ptr+3);	double zdot=*(X0_ptr+4);	double thetadot=*(X0_ptr+5);
		double v1=*(X0_ptr+6);		double v2=*(X0_ptr+7);		double v3=*(X0_ptr+8);
		double phi1=*(X0_ptr+9);	double phi2=*(X0_ptr+10);	double phi3=*(X0_ptr+11);
		double xc=*xc_ptr;			double d_theta=*d_theta_ptr;
		double a=*abc_ptr;			double b=*(abc_ptr+1);		double c=*(abc_ptr+2);
		*(F_ptr+k)=0;
		double checkflag=(double)rand()/RAND_MAX; //50% population has to align their signs in d_phi (random)
		for (int j=0; j<horizon; j++)
		{
			if (checkflag<=0.5)
			{
				if (xc<=*std_xc_ptr && d_theta<=0 && (phi1+*(d_phi_ptr+j*P+k))>=0) {
					if (phi1>=0 && *(d_phi_ptr+j*P+k)>=0)
						*(d_phi_ptr+j*P+k)=-*(d_phi_ptr+j*P+k);
					else if (phi1<=0 && *(d_phi_ptr+j*P+k)>=0)
						*(d_phi_ptr+j*P+k)=(double)rand()/RAND_MAX*(-phi1);
				}
				else if (xc<=*std_xc_ptr && d_theta>=0.09 && (phi1+*(d_phi_ptr+j*P+k))<=0) {
					if (phi1<=0 && *(d_phi_ptr+j*P+k)<=0)
						*(d_phi_ptr+j*P+k)=-*(d_phi_ptr+j*P+k);
					else if (phi1>=0 && *(d_phi_ptr+j*P+k)<=0)
						*(d_phi_ptr+j*P+k)=(double)rand()/RAND_MAX*(-phi1);
				}
				else if (xc<=*std_xc_ptr-0.3 && d_theta<0.09 && d_theta>=0 && (phi1+*(d_phi_ptr+j*P+k))>=0) {
					if (phi1>=0 && *(d_phi_ptr+j*P+k)>=0)
						*(d_phi_ptr+j*P+k)=-*(d_phi_ptr+j*P+k);
					else if (phi1<=0 && *(d_phi_ptr+j*P+k)>=0)
						*(d_phi_ptr+j*P+k)=(double)rand()/RAND_MAX*(-phi1);
				}
				else if (xc>=*std_xc_ptr && d_theta>=0 && (phi1+*(d_phi_ptr+j*P+k))<=0) {
					if (phi1<=0 && *(d_phi_ptr+j*P+k)<=0)
						*(d_phi_ptr+j*P+k)=-*(d_phi_ptr+j*P+k);
					else if (phi1>=0 && *(d_phi_ptr+j*P+k)<=0)
						*(d_phi_ptr+j*P+k)=(double)rand()/RAND_MAX*(-phi1);
				}
				else if (xc>=*std_xc_ptr && d_theta<=-0.09 && (phi1+*(d_phi_ptr+j*P+k))>=0) {
					if (phi1>=0 && *(d_phi_ptr+j*P+k)>=0)
						*(d_phi_ptr+j*P+k)=-*(d_phi_ptr+j*P+k);
					else if (phi1<=0 && *(d_phi_ptr+j*P+k)>=0)
						*(d_phi_ptr+j*P+k)=(double)rand()/RAND_MAX*(-phi1);
				}
				else if (xc>=*std_xc_ptr+0.3 && d_theta>-0.09 && d_theta<=0 && (phi1+*(d_phi_ptr+j*P+k))<=0){
					if (phi1<=0 && *(d_phi_ptr+j*P+k)<=0)
						*(d_phi_ptr+j*P+k)=-*(d_phi_ptr+j*P+k);
					else if (phi1>=0 && *(d_phi_ptr+j*P+k)<=0)
						*(d_phi_ptr+j*P+k)=(double)rand()/RAND_MAX*(-phi1);
				}
			}

			double pre_v1=v1;
			//v1=(7.82*Ts*Ts*(v1+*(d_v_ptr+j*P+k))+(2+2.064*Ts)*v2-v3)/deno_v;
			v1=v1+*(d_v_ptr+j*P+k);
			v3=v2; v2=pre_v1;
			double pre_phi1=phi1;
			//phi1=(252.2*Ts*Ts*(phi1)+(2+23.15*Ts)*phi2-phi3)/deno_phi+*(d_phi_ptr+j*P+k);
			phi1=phi1+*(d_phi_ptr+j*P+k);
			phi3=phi2; phi2=pre_phi1;
			double phit=((*(d_phi_ptr+j*P+k)+phi2)*deno_phi+phi3-(2+23.15*Ts)*phi2)/(252.3*Ts*Ts);
			if (phit>0.4) {
				phit=0.4;
				phi1=(252.3*Ts*Ts*phit+(2+23.15*Ts)*phi2-phi3)/deno_phi;
				*(d_phi_ptr+j*P+k)=phi1-phi2;
			}
			else if (phit<-0.4) {
				phit=-0.4;
				phi1=(252.3*Ts*Ts*phit+(2+23.15*Ts)*phi2-phi3)/deno_phi;
				*(d_phi_ptr+j*P+k)=phi1-phi2;
			}

			thetadot=-tan(phi1)*v1/1.28;
			theta=theta+thetadot*Ts;
			xdot=cos(theta)*v1;
			x=x+xdot*Ts;
			zdot=sin(theta)*v1;
			z=z+zdot*Ts;
			solve_xc_d_theta(&a, &b, &c, &x, &z, &theta, &xc, &d_theta);
			double acc=abs(v1*v1*tan(phi1)/1.28);
			double w_acc=(acc>1.6)?w_acc0:0;
			double w_v21=(v1<1.0)?10:0;
			*(F_ptr+k)=*(F_ptr+k)+w_xc*abs(xc-*std_xc_ptr)+w_d_theta*abs(d_theta)
				+w_acc*acc+w_v*abs(v1-*std_v0_ptr)+w_d_v*abs(*(d_v_ptr+j*P+k))+w_d_phi*abs(*(d_phi_ptr+j*P+k))
				+w_v21*abs(1.0-v1)+w_v*abs(10-v1);
		}
		*(F_ptr+k)=1/(1+*(F_ptr+k));
	}
}