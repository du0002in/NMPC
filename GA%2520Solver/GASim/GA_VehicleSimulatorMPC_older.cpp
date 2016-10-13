#define _USE_MATH_DEFINES // for C++ 
#include "mex.h"
 #include <windows.h>
 #include <stdio.h>
 #include <process.h>
 #include <string.h>
 #include <time.h>
 #include <cmath>
 /*These global variables will be accessible to both threads and will exist
*in the MEX function through multiple calls to the MEX function */
 static unsigned flag;
 double *xc, *d_theta, *x00, *z0, *theta0,*phi0, *v0, *dumy, *abc, log_v[40], log_phi[40], log_t[40];
 static HANDLE hThread=NULL;

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

 /*The second thread should not try to communicate with MATLAB at all. 
*This includes the use of mexPrintf, which attempts to print to the
*MATLAB command window. */
 unsigned __stdcall SecondThreadFunc( void* pArguments ) {
	 double new_xc, new_d_theta, a, b, c;
	 double x,z,theta,xdot,zdot,thetadot;
	 double dt, L_wheels;
	 double v_t, phi_t, phi_a0, phi_a1, phi_a2, v_a0, v_a1, v_a2;
	 char str1[10], str2[10];
	 FILE *fp_manuver, *fp_SMC_log, *fp_log_v, *fp_log_phi;
	 clock_t start_t, end_t;
	 v_t=0; phi_t=0; dt=0; phi_a0=*phi0; phi_a1=*phi0; phi_a2=*phi0;
	 v_a0=*v0; v_a1=*v0; v_a2=*v0;
	 theta = *theta0; x=*x00; z=*z0;
	 a=*abc; b=*(abc+1); c=*(abc+2); L_wheels=1.28;
	 /*for data logging and debugging only*/
	 /*if ( (fp_SMC_log = fopen ("D:\\Stereo\\Vehicle Simulator\\ActualPosition_log.txt","w")) != NULL ) {
		 fprintf (fp_SMC_log, "dt x z theta phi_a v_a new_xc new_d_theta phi_t v_t\n");
		 fclose (fp_SMC_log);
	 }*/
	 start_t = clock();
     while ( flag == 1 ) {

		 if ( (fp_manuver = fopen ("D:\\Stereo\\manuver.txt", "r")) != NULL ) {
			 if ( (fscanf (fp_manuver, "%s %lf %s %lf", str1, &v_t, str2, &phi_t)) != 0 ) {
					end_t = clock();
					dt = (double)(end_t-start_t) / CLOCKS_PER_SEC;
					start_t = clock();
					phi_a0=(252.2*dt*dt*phi_t+(2+23.15*dt)*phi_a1-phi_a2)/(1+23.15*dt+252.3*dt*dt);
					if (phi_a0>0.45) phi_a0=0.45; else if (phi_a0<-0.45) phi_a0=-0.45;
					phi_a2=phi_a1; phi_a1=phi_a0;

					v_a0=(7.82*dt*dt*v_t+(2+2.064*dt)*v_a1-v_a2)/(1+2.064*dt+7.818*dt*dt);
					if (v_a0<0) v_a0=0;
					v_a2=v_a1; v_a1=v_a0;

					thetadot = -tan(phi_a0) * v_a0 / L_wheels;
					theta = theta + thetadot * dt;
					xdot = cos(theta) * v_a0;
					x = x + xdot * dt;
					zdot = sin(theta) * v_a0;
					z = z + zdot * dt;
					solve_xc_d_theta( &a, &b, &c, &x, &z, &theta, &new_xc, &new_d_theta );
					Sleep(5L);
					//new_xc=-(x+1.5);
					//new_d_theta=theta-M_PI/2;
					/*
					if ((( fp_xc = fopen ("D:\\Stereo\\Vehicle Simulator\\act_xc.txt", "w")) != NULL) & (( fp_d_theta = fopen ("D:\\Stereo\\Vehicle Simulator\\act_d_theta.txt", "w")) != NULL )) {
						fprintf(fp_xc, "%4.3f", new_xc);
						fprintf(fp_d_theta, "%4.3f", new_d_theta);
						fclose(fp_xc); fclose(fp_d_theta);
					}
					
					if ((( fp_v = fopen ("D:\\Stereo\\Vehicle Simulator\\act_v.txt", "w")) != NULL) & (( fp_phi = fopen ("D:\\Stereo\\Vehicle Simulator\\act_phi.txt", "w")) != NULL )) {
						fprintf(fp_v, "%4.3f", v_a0);
						fprintf(fp_phi, "%4.3f", phi_a0);
						fclose(fp_v); fclose(fp_phi);
					}

					if ((( fp_v = fopen ("D:\\Stereo\\v_m.txt", "w")) != NULL) & (( fp_phi = fopen ("D:\\Stereo\\phai_m.txt", "w")) != NULL )) {
						fprintf(fp_v, "%4.3f", v_a0);
						fprintf(fp_phi, "%4.3f", phi_a0);
						fclose(fp_v); fclose(fp_phi);
					}

					if ((( fp_x = fopen ("D:\\Stereo\\Vehicle Simulator\\act_x.txt", "w")) != NULL) & (( fp_z = fopen ("D:\\Stereo\\Vehicle Simulator\\act_z.txt", "w")) != NULL) & (( fp_theta = fopen ("D:\\Stereo\\Vehicle Simulator\\act_theta.txt", "w")) != NULL )) {
						fprintf(fp_x, "%4.3f", x);
						fprintf(fp_z, "%4.3f", z);
						fprintf(fp_theta, "%4.3f", theta);
						fclose(fp_x);fclose(fp_z); fclose(fp_theta);
					}*/
					
					if ( (fp_SMC_log = fopen ("D:\\Stereo\\MPC\\GA Solver\\GASim\\GA_ActualPosition_log.txt","w")) != NULL ) {
						fprintf(fp_SMC_log, "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n", dt, x, z, theta, phi_a0, v_a0, new_xc, new_d_theta, phi_t, v_t);
						fclose (fp_SMC_log);
					}
					double cum_t=dt;
					int ind1=0, ind2=0;
					for (int i=0;i<40;i++) {
						cum_t=cum_t+log_t[-i+39];
						if (ind1==0) {
							if (cum_t>=0.1) {
								if (abs(cum_t-0.1)<abs(0.1-(cum_t-log_t[-i+39])))
									ind1=-i+39;
								else ind1=-i+39+1;
							}
						}
						if (ind2==0) {
							if (cum_t>=0.2) {
								if (abs(cum_t-0.2)<abs(0.2-(cum_t-log_t[-i+39])))
									ind2=-i+39;
								else ind2=-i+39+1;
								break;
							}
						}
					}
					for (int i=0;i<39;i++) {
						log_v[i]=log_v[i+1];
						log_phi[i]=log_phi[i+1];
						log_t[i]=log_t[i+1];
					}
					log_v[39]=v_a0;
					log_phi[39]=phi_a0;
					log_t[39]=dt;
					if (ind1!=0)
						ind1=ind1-1;
					if (ind2!=0)
						ind2=ind2-1;
					if ( (fp_log_v = fopen ("D:\\Stereo\\log_v_m.txt","w")) != NULL ) {
						fprintf(fp_log_v, "%6.4f\n%6.4f\n%6.4f\n",log_v[ind2],log_v[ind1],log_v[39]);
						fclose(fp_log_v);
					}
					if ( (fp_log_phi = fopen ("D:\\Stereo\\log_phai_m.txt","w")) != NULL ) {
						fprintf(fp_log_phi, "%6.4f\n%6.4f\n%6.4f\n",log_phi[ind2],log_phi[ind1],log_phi[39]);
						fclose(fp_log_phi);
					}
			 }
			fclose(fp_manuver);
		 }
	 }

     _endthreadex( 0 );
     return 0;
 }
 void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[]) {
     unsigned threadID, threadID_dummy;
	 //double *dumy;
     char *cmd;
     /*  check for proper number of arguments */
     //if(nrhs!=4)
     //    mexErrMsgTxt("Four input required.");
     /* check to make sure the first input argument is a string */
     //if (!mxIsChar(prhs[0]))
     //    mexErrMsgTxt("Input must be a string.");
     /* get command from first input */
     cmd = mxArrayToString(prhs[0]);
	 xc=mxGetPr(prhs[1]);
	 d_theta=mxGetPr(prhs[2]);
	 x00=mxGetPr(prhs[3]);
	 z0=mxGetPr(prhs[4]);
	 theta0=mxGetPr(prhs[5]);
	 phi0=mxGetPr(prhs[6]);
	 v0=mxGetPr(prhs[7]);
	 abc=mxGetPr(prhs[8]);
	 //x00=mxGetPr(prhs[8]);
	 // abc=mxGetPr(prhs[3]);

     /* if command is "Init", start the thread, otherwise wait for thread
      * to complete */
     if (!strcmp(cmd,"Ini")) {
		 for (int i=0; i<40; i++) {
			 log_v[i]=0;
			 log_phi[i]=0;
			 log_t[i]=0;
		 }
         /* make sure thread was not started using "mex locked" state */
         if (mexIsLocked())
             mexErrMsgTxt("Thread already initialized.");
         /* lock thread so that no-one accidentally clears function */
         mexLock();
         /* Create the second thread. */
         mexPrintf( "Creating Vehicle Simulator thread...\n" );
         flag = 1;
         hThread = (HANDLE)_beginthreadex( NULL, 0, &SecondThreadFunc, NULL, 0, &threadID );

		 // hThread_dumy = (HANDLE)_beginthreadex( NULL, 0, &DummyThreadFunc, NULL, 0, &threadID_dummy );
     }
	 else if (!strcmp(cmd,"Con")) {
		 if (!mexIsLocked())
             mexErrMsgTxt("Thread not initialized yet."); /*This function will return control to MATLAB*/
		 flag = 1;
	 }
     else if (!strcmp(cmd,"End")) {
         /* make sure that the thread was started using "mex locked" status*/
         if (!mexIsLocked())
             mexErrMsgTxt("Thread not initialized yet."); /*This function will return control to MATLAB*/
		 flag = 0;
		 /* wait for thread to finish and get result */
         WaitForSingleObject( hThread, INFINITE );
		 // WaitForSingleObject( hThread_dumy, INFINITE );
         mexPrintf( "Terminated Vehicle Simulator thread...\n" );
         /* Destroy the thread object, free memory, and unlock mex. */
         CloseHandle( hThread );
		 // CloseHandle( hThread_dumy );
         mexUnlock();
     }
     return;
 }