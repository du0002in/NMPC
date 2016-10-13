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
 double x_final, z_final, theta_final;
 static HANDLE hThread=NULL, hThread_dumy=NULL;

 /*The second thread should not try to communicate with MATLAB at all. 
*This includes the use of mexPrintf, which attempts to print to the
*MATLAB command window. */
 unsigned __stdcall SecondThreadFunc( void* pArguments ) {
	 FILE *fp_SMC_log,*fp_m;
	 double v_m, phi_m, xdot,zdot,thetadot,pre_v_m, pre_phi_m, dt;
	 double L_wheels;
	 double x,z,theta;
	 double dumy[10];
	 clock_t start_t, end_t;
	 L_wheels=1.28;
	 start_t = clock();
	 x=0.0; z=0.0; theta=M_PI * 0.5;
	 v_m=0; phi_m=0; pre_v_m=0; pre_phi_m=0;

	 if (( fp_m = fopen ("D:\\Stereo\\t_phi_v_m.txt", "w")) != NULL) {
						fclose(fp_m);
		}

     while ( flag == 1 ) {
		 if ((fp_SMC_log = fopen ("D:\\Stereo\\MPC\\GA Solver\\GASim\\GA_ActualPosition_log.txt","r")) != NULL) {
			 if ((fscanf(fp_SMC_log,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				 &dumy[0],&dumy[1],&dumy[2],&dumy[3],&dumy[4],&dumy[5],&dumy[6],&dumy[7],&dumy[8],&dumy[9]) != 0)) {
					 v_m=dumy[5];
					 phi_m=dumy[4];
					 pre_v_m=v_m;
					 pre_phi_m=phi_m;
			 }
			 else {
				 v_m=pre_v_m;
				 phi_m=pre_phi_m;
			 }
			 fclose(fp_SMC_log);
		 }
		 Sleep(9L);
		 end_t = clock();
		 dt = (double)(end_t-start_t) / CLOCKS_PER_SEC;
		 start_t = clock();
		 thetadot = -tan(phi_m) * v_m / L_wheels;
		 theta = theta + thetadot * dt;
		 xdot = cos(theta) * v_m;
		 x = x + xdot * dt;
		 zdot = sin(theta) * v_m;
		 z = z + zdot * dt; 
		 if (( fp_m = fopen ("D:\\Stereo\\t_phi_v_m.txt", "a")) != NULL) {
						fprintf(fp_m, "%4.3f %4.3f %4.3f\n", dt, phi_m, v_m);
						fclose(fp_m);
					}
	 }
	 x_final=x;
	 z_final=z;
	 theta_final=theta;
     _endthreadex( 0 );
     return 0;
 }
 void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
     unsigned threadID, threadID_dummy;
     char *cmd;
     double *xztheta_final_ptr;

     cmd = mxArrayToString(prhs[0]);

	 plhs[0]=mxCreateDoubleMatrix(1, 3, mxREAL);
	 xztheta_final_ptr=mxGetPr(plhs[0]);

     /* if command is "Init", start the thread, otherwise wait for thread
      * to complete */
     if (!strcmp(cmd,"Ini")) {
         /* make sure thread was not started using "mex locked" state */
         if (mexIsLocked())
             mexErrMsgTxt("Thread already initialized.");
         /* lock thread so that no-one accidentally clears function */
         mexLock();
         /* Create the second thread. */
		 //mexPrintf( "Creating Vehicle Simulator thread...\n" );
         flag = 1;
         hThread = (HANDLE)_beginthreadex( NULL, 0, &SecondThreadFunc, NULL, 0, &threadID );
		 *xztheta_final_ptr=0.0;
		 *(xztheta_final_ptr+1)=0.0;
		 *(xztheta_final_ptr+2)=0.0;
     }
	
     else if (!strcmp(cmd,"End")) {
         /* make sure that the thread was started using "mex locked" status*/
         if (!mexIsLocked())
             mexErrMsgTxt("Thread not initialized yet."); /*This function will return control to MATLAB*/
		 flag = 0;
		 /* wait for thread to finish and get result */
         WaitForSingleObject( hThread, INFINITE );
		 /*assign value*/
		 *xztheta_final_ptr=x_final;
		 *(xztheta_final_ptr+1)=z_final;
		 *(xztheta_final_ptr+2)=theta_final;
		 //mexPrintf( "Terminated Vehicle Simulator thread...\n" );
         /* Destroy the thread object, free memory, and unlock mex. */
         CloseHandle( hThread );
		 // CloseHandle( hThread_dumy );
         mexUnlock();
     }
     return;
 }