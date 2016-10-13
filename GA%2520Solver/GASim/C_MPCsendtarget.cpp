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
 static unsigned flag, cycle, horizon;
 double *V_Phi, *Ts;
 static HANDLE hThread=NULL;

 /*The second thread should not try to communicate with MATLAB at all. 
*This includes the use of mexPrintf, which attempts to print to the
*MATLAB command window. */
 unsigned __stdcall SecondThreadFunc( void* pArguments ) {
	 double dt, cum_t;
	 FILE *fp_manuver;
	 clock_t start_t, end_t;
	 int i,k;
	 double *v_t=new double[horizon];
	 double *phi_t=new double[horizon];
	 
	 while (flag==1) {
		 cum_t=0;
		 cycle=0;
		 i=-1;
		 for (int j=0; j<horizon; j++) {
			 v_t[j]=*(V_Phi+2*j);
			 phi_t[j]=*(V_Phi+2*j+1);
		 }
		 while (cycle==0) {
			 cum_t=0;
			 start_t=clock();
			 i=i+1;
			 i=(i<horizon)?i:(horizon-1);
			 if ( (fp_manuver = fopen ("D:\\Stereo\\manuver.txt", "w")) != NULL ) {
				 fprintf(fp_manuver, "tv %4.3f\ntp %4.3f", v_t[i], phi_t[i]);
				 fclose(fp_manuver);
			 }
			 end_t=clock();
			 dt = (double)(end_t-start_t) / CLOCKS_PER_SEC;
			 cum_t=cum_t+dt;
			 start_t=clock();
			 while (cum_t<*Ts & cycle==0) {
				 Sleep(1);
				 end_t=clock();
				 dt = (double)(end_t-start_t) / CLOCKS_PER_SEC;
				 cum_t=dt;
			 }
		 }
	 }
	 delete [] v_t;
	 delete [] phi_t;
     _endthreadex( 0 );
     return 0;
 }
 void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[]) {
     unsigned threadID;
     char *cmd;

     cmd = mxArrayToString(prhs[0]);
	 V_Phi=mxGetPr(prhs[1]);
	 Ts=mxGetPr(prhs[2]);
	 horizon=mxGetN(prhs[1]);

     /* if command is "Init", start the thread, otherwise wait for thread
      * to complete */
     if (!strcmp(cmd,"Ini")) {
         /* make sure thread was not started using "mex locked" state */
         if (mexIsLocked())
             mexErrMsgTxt("Thread already initialized.");
         /* lock thread so that no-one accidentally clears function */
         mexLock();
         /* Create the second thread. */
         mexPrintf( "Creating SendTarget thread...\n" );
         flag = 1;
		 cycle = 1;
         hThread = (HANDLE)_beginthreadex( NULL, 0, &SecondThreadFunc, NULL, 0, &threadID );

		 // hThread_dumy = (HANDLE)_beginthreadex( NULL, 0, &DummyThreadFunc, NULL, 0, &threadID_dummy );
     }
	 else if (!strcmp(cmd,"Con")) {
		 if (!mexIsLocked())
             mexErrMsgTxt("Thread not initialized yet."); /*This function will return control to MATLAB*/
		 flag = 1;
		 cycle = 1;
	 }
     else if (!strcmp(cmd,"End")) {
         /* make sure that the thread was started using "mex locked" status*/
         if (!mexIsLocked())
             mexErrMsgTxt("Thread not initialized yet."); /*This function will return control to MATLAB*/
		 flag = 0; cycle=1;
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