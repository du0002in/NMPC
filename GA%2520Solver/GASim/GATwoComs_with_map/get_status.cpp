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
 double status;
 static HANDLE hThread=NULL;

 /*The second thread should not try to communicate with MATLAB at all. 
*This includes the use of mexPrintf, which attempts to print to the
*MATLAB command window. */
 unsigned __stdcall SecondThreadFunc( void* pArguments ) {

	 FILE *fp_esposition;
	 double status_dumy;

	 while (flag==1) {
		 if ((fp_esposition = fopen ("\\Stereo\\MPC\\MultiSession\\location.txt","r")) != NULL) {
			 fscanf(fp_esposition,"%lf",&status_dumy);
			 fclose(fp_esposition);
		 }
		 Sleep(1L);
		 status=status_dumy;
	 }
	 //status=status_dumy;
     _endthreadex( 0 );
     return 0;
 }
 void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
     unsigned threadID;
     char *cmd;
	 double *status_ptr;

     cmd = mxArrayToString(prhs[0]);
	 plhs[0]=mxCreateDoubleMatrix(1, 1, mxREAL);
	 status_ptr=mxGetPr(plhs[0]);
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
		 *status_ptr=0.0;
     }
	 else if (!strcmp(cmd,"Con")) {
		 if (!mexIsLocked())
             mexErrMsgTxt("Thread not initialized yet."); /*This function will return control to MATLAB*/
		 flag = 1;
		 *status_ptr=status;
	 }	
     else if (!strcmp(cmd,"End")) {
         /* make sure that the thread was started using "mex locked" status*/
         if (!mexIsLocked())
             mexErrMsgTxt("Thread not initialized yet."); /*This function will return control to MATLAB*/
		 flag = 0;
		 /* wait for thread to finish and get result */
         WaitForSingleObject( hThread, INFINITE );
		 *status_ptr=status;
		 //mexPrintf( "Terminated get_status thread...\n" );
         /* Destroy the thread object, free memory, and unlock mex. */
         CloseHandle( hThread );
		 // CloseHandle( hThread_dumy );
         mexUnlock();
     }
     return;
 }