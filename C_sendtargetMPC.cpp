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
 static unsigned flag, new_cycle;
 double *phi_t_ptr, *v_t_ptr;
 int horizon;
 static HANDLE hThread=NULL;

 /*The second thread should not try to communicate with MATLAB at all. 
*This includes the use of mexPrintf, which attempts to print to the
*MATLAB command window. */
 unsigned __stdcall SecondThreadFunc( void* pArguments ) {
	 int horizon_thrd;
	 horizon_thrd=horizon;
	 double *phi_t=new double[horizon_thrd];
	 double *v_t=new double[horizon_thrd];
	 double dt=0; int counter=0;
	 FILE *fp_manuver;
	 clock_t start_t, end_t;
	 start_t = clock();
	 while (flag==1) {
		 if (new_cycle==1) {
			 new_cycle=0;
			 for (int i=0;i<horizon_thrd;i++) {
				 phi_t[i]=*(phi_t_ptr+i);
				 v_t[i]=*(v_t_ptr+i);
			 }
			 dt=0; counter=0;
			 if (( fp_manuver = fopen ("\\Stereo\\manuver.txt", "w")) != NULL ) {
					 fprintf(fp_manuver, "tv %4.3f\ntp %4.3f", v_t[counter], phi_t[counter]);
					 fclose (fp_manuver);
				 }
			 counter=1;
		 }

		 end_t = clock();
		 dt = (double)(end_t-start_t) / CLOCKS_PER_SEC+dt;
		 start_t = clock();

		 if (dt<0.03) {
			 Sleep(1L);
		 }
		 else {
			 dt=0;
			 if (( fp_manuver = fopen ("\\Stereo\\manuver.txt", "w")) != NULL ) {
					 fprintf(fp_manuver, "tv %4.3f\ntp %4.3f", v_t[counter], phi_t[counter]);
					 fclose (fp_manuver);
				 }
			 counter=(counter>=7)?7:(counter+1);
		 }
		 
	 }
	 delete [] v_t;
	 delete [] phi_t;
     _endthreadex( 0 );
     return 0;
 }
 void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
     unsigned threadID;
     char *cmd;

     cmd = mxArrayToString(prhs[0]);
	 phi_t_ptr=mxGetPr(prhs[1]);
	 v_t_ptr=mxGetPr(prhs[2]);

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
		 new_cycle=1;
		 horizon=mxGetN(prhs[1]);
         hThread = (HANDLE)_beginthreadex( NULL, 0, &SecondThreadFunc, NULL, 0, &threadID );
     }
	 else if (!strcmp(cmd,"Con")) {
		 if (!mexIsLocked())
             mexErrMsgTxt("Thread not initialized yet."); /*This function will return control to MATLAB*/
		 flag = 1;
		 new_cycle = 1;
	 }
     else if (!strcmp(cmd,"End")) {
         /* make sure that the thread was started using "mex locked" status*/
         if (!mexIsLocked())
             mexErrMsgTxt("Thread not initialized yet."); /*This function will return control to MATLAB*/
		 flag = 0;
		 /* wait for thread to finish and get result */
         WaitForSingleObject( hThread, INFINITE );
		 //mexPrintf( "Terminated Vehicle Simulator thread...\n" );
         /* Destroy the thread object, free memory, and unlock mex. */
         CloseHandle( hThread );
		 // CloseHandle( hThread_dumy );
         mexUnlock();
     }
     return;
 }