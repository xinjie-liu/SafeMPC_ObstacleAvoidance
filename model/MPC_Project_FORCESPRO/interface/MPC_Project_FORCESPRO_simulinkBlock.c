/*
MPC_Project_FORCESPRO : A fast customized optimization solver.

Copyright (C) 2013-2021 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME MPC_Project_FORCESPRO_simulinkBlock

#include "simstruc.h"

/* For compatibility with Microsoft Visual Studio 2015 */
#if _MSC_VER >= 1900
FILE _iob[3];
FILE * __cdecl __iob_func(void)
{
	_iob[0] = *stdin;
	_iob[1] = *stdout;
	_iob[2] = *stderr;
	return _iob;
}
#endif

/* include FORCESPRO functions and defs */
#include "../include/MPC_Project_FORCESPRO.h" 

/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef MPC_Project_FORCESPROinterface_float MPC_Project_FORCESPROnmpc_float;





/*====================*
 * S-function methods *
 *====================*/
/* Function: mdlInitializeSizes =========================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
	{
		return; /* Parameter mismatch will be reported by Simulink */
    }

	/* initialize size of continuous and discrete states to zero */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

	/* initialize input ports - there are 79 in total */
    if (!ssSetNumInputPorts(S, 79)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 3, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 3, 5);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 3, 5);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 3, 5);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/
	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 3, 5);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/
	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 3, 5);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/
	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 3, 5);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/
	
	/* Input Port 7 */
    ssSetInputPortMatrixDimensions(S,  7, 3, 5);
    ssSetInputPortDataType(S, 7, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 7, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 7, 1); /*direct input signal access*/
	
	/* Input Port 8 */
    ssSetInputPortMatrixDimensions(S,  8, 3, 5);
    ssSetInputPortDataType(S, 8, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 8, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 8, 1); /*direct input signal access*/
	
	/* Input Port 9 */
    ssSetInputPortMatrixDimensions(S,  9, 3, 5);
    ssSetInputPortDataType(S, 9, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 9, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 9, 1); /*direct input signal access*/
	
	/* Input Port 10 */
    ssSetInputPortMatrixDimensions(S,  10, 3, 5);
    ssSetInputPortDataType(S, 10, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 10, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 10, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 10, 1); /*direct input signal access*/
	
	/* Input Port 11 */
    ssSetInputPortMatrixDimensions(S,  11, 3, 5);
    ssSetInputPortDataType(S, 11, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 11, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 11, 1); /*direct input signal access*/
	
	/* Input Port 12 */
    ssSetInputPortMatrixDimensions(S,  12, 3, 5);
    ssSetInputPortDataType(S, 12, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 12, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 12, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 12, 1); /*direct input signal access*/
	
	/* Input Port 13 */
    ssSetInputPortMatrixDimensions(S,  13, 3, 5);
    ssSetInputPortDataType(S, 13, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 13, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 13, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 13, 1); /*direct input signal access*/
	
	/* Input Port 14 */
    ssSetInputPortMatrixDimensions(S,  14, 3, 5);
    ssSetInputPortDataType(S, 14, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 14, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 14, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 14, 1); /*direct input signal access*/
	
	/* Input Port 15 */
    ssSetInputPortMatrixDimensions(S,  15, 3, 5);
    ssSetInputPortDataType(S, 15, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 15, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 15, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 15, 1); /*direct input signal access*/
	
	/* Input Port 16 */
    ssSetInputPortMatrixDimensions(S,  16, 3, 5);
    ssSetInputPortDataType(S, 16, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 16, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 16, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 16, 1); /*direct input signal access*/
	
	/* Input Port 17 */
    ssSetInputPortMatrixDimensions(S,  17, 3, 5);
    ssSetInputPortDataType(S, 17, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 17, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 17, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 17, 1); /*direct input signal access*/
	
	/* Input Port 18 */
    ssSetInputPortMatrixDimensions(S,  18, 3, 5);
    ssSetInputPortDataType(S, 18, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 18, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 18, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 18, 1); /*direct input signal access*/
	
	/* Input Port 19 */
    ssSetInputPortMatrixDimensions(S,  19, 3, 5);
    ssSetInputPortDataType(S, 19, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 19, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 19, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 19, 1); /*direct input signal access*/
	
	/* Input Port 20 */
    ssSetInputPortMatrixDimensions(S,  20, 3, 5);
    ssSetInputPortDataType(S, 20, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 20, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 20, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 20, 1); /*direct input signal access*/
	
	/* Input Port 21 */
    ssSetInputPortMatrixDimensions(S,  21, 3, 5);
    ssSetInputPortDataType(S, 21, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 21, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 21, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 21, 1); /*direct input signal access*/
	
	/* Input Port 22 */
    ssSetInputPortMatrixDimensions(S,  22, 3, 5);
    ssSetInputPortDataType(S, 22, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 22, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 22, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 22, 1); /*direct input signal access*/
	
	/* Input Port 23 */
    ssSetInputPortMatrixDimensions(S,  23, 3, 5);
    ssSetInputPortDataType(S, 23, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 23, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 23, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 23, 1); /*direct input signal access*/
	
	/* Input Port 24 */
    ssSetInputPortMatrixDimensions(S,  24, 3, 5);
    ssSetInputPortDataType(S, 24, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 24, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 24, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 24, 1); /*direct input signal access*/
	
	/* Input Port 25 */
    ssSetInputPortMatrixDimensions(S,  25, 3, 5);
    ssSetInputPortDataType(S, 25, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 25, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 25, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 25, 1); /*direct input signal access*/
	
	/* Input Port 26 */
    ssSetInputPortMatrixDimensions(S,  26, 3, 5);
    ssSetInputPortDataType(S, 26, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 26, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 26, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 26, 1); /*direct input signal access*/
	
	/* Input Port 27 */
    ssSetInputPortMatrixDimensions(S,  27, 3, 5);
    ssSetInputPortDataType(S, 27, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 27, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 27, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 27, 1); /*direct input signal access*/
	
	/* Input Port 28 */
    ssSetInputPortMatrixDimensions(S,  28, 3, 5);
    ssSetInputPortDataType(S, 28, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 28, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 28, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 28, 1); /*direct input signal access*/
	
	/* Input Port 29 */
    ssSetInputPortMatrixDimensions(S,  29, 3, 5);
    ssSetInputPortDataType(S, 29, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 29, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 29, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 29, 1); /*direct input signal access*/
	
	/* Input Port 30 */
    ssSetInputPortMatrixDimensions(S,  30, 3, 5);
    ssSetInputPortDataType(S, 30, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 30, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 30, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 30, 1); /*direct input signal access*/
	
	/* Input Port 31 */
    ssSetInputPortMatrixDimensions(S,  31, 3, 5);
    ssSetInputPortDataType(S, 31, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 31, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 31, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 31, 1); /*direct input signal access*/
	
	/* Input Port 32 */
    ssSetInputPortMatrixDimensions(S,  32, 3, 5);
    ssSetInputPortDataType(S, 32, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 32, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 32, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 32, 1); /*direct input signal access*/
	
	/* Input Port 33 */
    ssSetInputPortMatrixDimensions(S,  33, 3, 5);
    ssSetInputPortDataType(S, 33, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 33, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 33, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 33, 1); /*direct input signal access*/
	
	/* Input Port 34 */
    ssSetInputPortMatrixDimensions(S,  34, 3, 5);
    ssSetInputPortDataType(S, 34, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 34, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 34, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 34, 1); /*direct input signal access*/
	
	/* Input Port 35 */
    ssSetInputPortMatrixDimensions(S,  35, 3, 5);
    ssSetInputPortDataType(S, 35, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 35, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 35, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 35, 1); /*direct input signal access*/
	
	/* Input Port 36 */
    ssSetInputPortMatrixDimensions(S,  36, 3, 5);
    ssSetInputPortDataType(S, 36, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 36, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 36, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 36, 1); /*direct input signal access*/
	
	/* Input Port 37 */
    ssSetInputPortMatrixDimensions(S,  37, 3, 5);
    ssSetInputPortDataType(S, 37, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 37, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 37, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 37, 1); /*direct input signal access*/
	
	/* Input Port 38 */
    ssSetInputPortMatrixDimensions(S,  38, 3, 5);
    ssSetInputPortDataType(S, 38, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 38, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 38, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 38, 1); /*direct input signal access*/
	
	/* Input Port 39 */
    ssSetInputPortMatrixDimensions(S,  39, 3, 5);
    ssSetInputPortDataType(S, 39, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 39, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 39, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 39, 1); /*direct input signal access*/
	
	/* Input Port 40 */
    ssSetInputPortMatrixDimensions(S,  40, 5, 1);
    ssSetInputPortDataType(S, 40, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 40, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 40, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 40, 1); /*direct input signal access*/
	
	/* Input Port 41 */
    ssSetInputPortMatrixDimensions(S,  41, 5, 1);
    ssSetInputPortDataType(S, 41, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 41, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 41, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 41, 1); /*direct input signal access*/
	
	/* Input Port 42 */
    ssSetInputPortMatrixDimensions(S,  42, 5, 1);
    ssSetInputPortDataType(S, 42, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 42, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 42, 1); /*direct input signal access*/
	
	/* Input Port 43 */
    ssSetInputPortMatrixDimensions(S,  43, 5, 1);
    ssSetInputPortDataType(S, 43, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 43, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 43, 1); /*direct input signal access*/
	
	/* Input Port 44 */
    ssSetInputPortMatrixDimensions(S,  44, 5, 1);
    ssSetInputPortDataType(S, 44, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 44, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 44, 1); /*direct input signal access*/
	
	/* Input Port 45 */
    ssSetInputPortMatrixDimensions(S,  45, 5, 1);
    ssSetInputPortDataType(S, 45, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 45, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 45, 1); /*direct input signal access*/
	
	/* Input Port 46 */
    ssSetInputPortMatrixDimensions(S,  46, 5, 1);
    ssSetInputPortDataType(S, 46, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 46, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 46, 1); /*direct input signal access*/
	
	/* Input Port 47 */
    ssSetInputPortMatrixDimensions(S,  47, 5, 1);
    ssSetInputPortDataType(S, 47, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 47, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 47, 1); /*direct input signal access*/
	
	/* Input Port 48 */
    ssSetInputPortMatrixDimensions(S,  48, 5, 1);
    ssSetInputPortDataType(S, 48, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 48, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 48, 1); /*direct input signal access*/
	
	/* Input Port 49 */
    ssSetInputPortMatrixDimensions(S,  49, 5, 1);
    ssSetInputPortDataType(S, 49, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 49, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 49, 1); /*direct input signal access*/
	
	/* Input Port 50 */
    ssSetInputPortMatrixDimensions(S,  50, 5, 1);
    ssSetInputPortDataType(S, 50, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 50, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 50, 1); /*direct input signal access*/
	
	/* Input Port 51 */
    ssSetInputPortMatrixDimensions(S,  51, 5, 1);
    ssSetInputPortDataType(S, 51, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 51, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 51, 1); /*direct input signal access*/
	
	/* Input Port 52 */
    ssSetInputPortMatrixDimensions(S,  52, 5, 1);
    ssSetInputPortDataType(S, 52, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 52, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 52, 1); /*direct input signal access*/
	
	/* Input Port 53 */
    ssSetInputPortMatrixDimensions(S,  53, 5, 1);
    ssSetInputPortDataType(S, 53, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 53, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 53, 1); /*direct input signal access*/
	
	/* Input Port 54 */
    ssSetInputPortMatrixDimensions(S,  54, 5, 1);
    ssSetInputPortDataType(S, 54, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 54, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 54, 1); /*direct input signal access*/
	
	/* Input Port 55 */
    ssSetInputPortMatrixDimensions(S,  55, 5, 1);
    ssSetInputPortDataType(S, 55, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 55, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 55, 1); /*direct input signal access*/
	
	/* Input Port 56 */
    ssSetInputPortMatrixDimensions(S,  56, 5, 1);
    ssSetInputPortDataType(S, 56, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 56, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 56, 1); /*direct input signal access*/
	
	/* Input Port 57 */
    ssSetInputPortMatrixDimensions(S,  57, 5, 1);
    ssSetInputPortDataType(S, 57, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 57, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 57, 1); /*direct input signal access*/
	
	/* Input Port 58 */
    ssSetInputPortMatrixDimensions(S,  58, 5, 1);
    ssSetInputPortDataType(S, 58, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 58, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 58, 1); /*direct input signal access*/
	
	/* Input Port 59 */
    ssSetInputPortMatrixDimensions(S,  59, 5, 1);
    ssSetInputPortDataType(S, 59, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 59, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 59, 1); /*direct input signal access*/
	
	/* Input Port 60 */
    ssSetInputPortMatrixDimensions(S,  60, 5, 1);
    ssSetInputPortDataType(S, 60, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 60, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 60, 1); /*direct input signal access*/
	
	/* Input Port 61 */
    ssSetInputPortMatrixDimensions(S,  61, 5, 1);
    ssSetInputPortDataType(S, 61, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 61, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 61, 1); /*direct input signal access*/
	
	/* Input Port 62 */
    ssSetInputPortMatrixDimensions(S,  62, 5, 1);
    ssSetInputPortDataType(S, 62, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 62, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 62, 1); /*direct input signal access*/
	
	/* Input Port 63 */
    ssSetInputPortMatrixDimensions(S,  63, 5, 1);
    ssSetInputPortDataType(S, 63, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 63, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 63, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 63, 1); /*direct input signal access*/
	
	/* Input Port 64 */
    ssSetInputPortMatrixDimensions(S,  64, 5, 1);
    ssSetInputPortDataType(S, 64, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 64, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 64, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 64, 1); /*direct input signal access*/
	
	/* Input Port 65 */
    ssSetInputPortMatrixDimensions(S,  65, 5, 1);
    ssSetInputPortDataType(S, 65, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 65, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 65, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 65, 1); /*direct input signal access*/
	
	/* Input Port 66 */
    ssSetInputPortMatrixDimensions(S,  66, 5, 1);
    ssSetInputPortDataType(S, 66, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 66, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 66, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 66, 1); /*direct input signal access*/
	
	/* Input Port 67 */
    ssSetInputPortMatrixDimensions(S,  67, 5, 1);
    ssSetInputPortDataType(S, 67, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 67, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 67, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 67, 1); /*direct input signal access*/
	
	/* Input Port 68 */
    ssSetInputPortMatrixDimensions(S,  68, 5, 1);
    ssSetInputPortDataType(S, 68, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 68, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 68, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 68, 1); /*direct input signal access*/
	
	/* Input Port 69 */
    ssSetInputPortMatrixDimensions(S,  69, 5, 1);
    ssSetInputPortDataType(S, 69, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 69, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 69, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 69, 1); /*direct input signal access*/
	
	/* Input Port 70 */
    ssSetInputPortMatrixDimensions(S,  70, 5, 1);
    ssSetInputPortDataType(S, 70, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 70, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 70, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 70, 1); /*direct input signal access*/
	
	/* Input Port 71 */
    ssSetInputPortMatrixDimensions(S,  71, 5, 1);
    ssSetInputPortDataType(S, 71, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 71, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 71, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 71, 1); /*direct input signal access*/
	
	/* Input Port 72 */
    ssSetInputPortMatrixDimensions(S,  72, 5, 1);
    ssSetInputPortDataType(S, 72, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 72, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 72, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 72, 1); /*direct input signal access*/
	
	/* Input Port 73 */
    ssSetInputPortMatrixDimensions(S,  73, 5, 1);
    ssSetInputPortDataType(S, 73, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 73, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 73, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 73, 1); /*direct input signal access*/
	
	/* Input Port 74 */
    ssSetInputPortMatrixDimensions(S,  74, 5, 1);
    ssSetInputPortDataType(S, 74, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 74, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 74, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 74, 1); /*direct input signal access*/
	
	/* Input Port 75 */
    ssSetInputPortMatrixDimensions(S,  75, 5, 1);
    ssSetInputPortDataType(S, 75, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 75, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 75, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 75, 1); /*direct input signal access*/
	
	/* Input Port 76 */
    ssSetInputPortMatrixDimensions(S,  76, 5, 1);
    ssSetInputPortDataType(S, 76, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 76, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 76, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 76, 1); /*direct input signal access*/
	
	/* Input Port 77 */
    ssSetInputPortMatrixDimensions(S,  77, 5, 1);
    ssSetInputPortDataType(S, 77, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 77, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 77, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 77, 1); /*direct input signal access*/
	
	/* Input Port 78 */
    ssSetInputPortMatrixDimensions(S,  78, 5, 1);
    ssSetInputPortDataType(S, 78, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 78, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 78, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 78, 1); /*direct input signal access*/
 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 50, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */


	/* set sampling time */
    ssSetNumSampleTimes(S, 1);

	/* set internal memory of block */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
    /* ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
		             SS_OPTION_WORKS_WITH_CODE_REUSE)); */
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );

	
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S, 
                                         int_T            port,
                                         const DimsInfo_T *dimsInfo)
{
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
static void mdlSetOutputPortDimensionInfo(SimStruct        *S, 
                                          int_T            port, 
                                          const DimsInfo_T *dimsInfo)
{
    if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif
# define MDL_SET_INPUT_PORT_FRAME_DATA
static void mdlSetInputPortFrameData(SimStruct  *S, 
                                     int_T      port,
                                     Frame_T    frameData)
{
    ssSetInputPortFrameData(S, port, frameData);
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
    ssSetInputPortDataType( S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}





/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
	solver_int32_default i, j, k;
	
	/* file pointer for printing */
	FILE *fp = NULL;

	/* Simulink data */
	const real_T *xinit = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *linear_model1 = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *linear_model2 = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *linear_model3 = (const real_T*) ssGetInputPortSignal(S,3);
	const real_T *linear_model4 = (const real_T*) ssGetInputPortSignal(S,4);
	const real_T *linear_model5 = (const real_T*) ssGetInputPortSignal(S,5);
	const real_T *linear_model6 = (const real_T*) ssGetInputPortSignal(S,6);
	const real_T *linear_model7 = (const real_T*) ssGetInputPortSignal(S,7);
	const real_T *linear_model8 = (const real_T*) ssGetInputPortSignal(S,8);
	const real_T *linear_model9 = (const real_T*) ssGetInputPortSignal(S,9);
	const real_T *linear_model10 = (const real_T*) ssGetInputPortSignal(S,10);
	const real_T *linear_model11 = (const real_T*) ssGetInputPortSignal(S,11);
	const real_T *linear_model12 = (const real_T*) ssGetInputPortSignal(S,12);
	const real_T *linear_model13 = (const real_T*) ssGetInputPortSignal(S,13);
	const real_T *linear_model14 = (const real_T*) ssGetInputPortSignal(S,14);
	const real_T *linear_model15 = (const real_T*) ssGetInputPortSignal(S,15);
	const real_T *linear_model16 = (const real_T*) ssGetInputPortSignal(S,16);
	const real_T *linear_model17 = (const real_T*) ssGetInputPortSignal(S,17);
	const real_T *linear_model18 = (const real_T*) ssGetInputPortSignal(S,18);
	const real_T *linear_model19 = (const real_T*) ssGetInputPortSignal(S,19);
	const real_T *linear_model20 = (const real_T*) ssGetInputPortSignal(S,20);
	const real_T *linear_model21 = (const real_T*) ssGetInputPortSignal(S,21);
	const real_T *linear_model22 = (const real_T*) ssGetInputPortSignal(S,22);
	const real_T *linear_model23 = (const real_T*) ssGetInputPortSignal(S,23);
	const real_T *linear_model24 = (const real_T*) ssGetInputPortSignal(S,24);
	const real_T *linear_model25 = (const real_T*) ssGetInputPortSignal(S,25);
	const real_T *linear_model26 = (const real_T*) ssGetInputPortSignal(S,26);
	const real_T *linear_model27 = (const real_T*) ssGetInputPortSignal(S,27);
	const real_T *linear_model28 = (const real_T*) ssGetInputPortSignal(S,28);
	const real_T *linear_model29 = (const real_T*) ssGetInputPortSignal(S,29);
	const real_T *linear_model30 = (const real_T*) ssGetInputPortSignal(S,30);
	const real_T *linear_model31 = (const real_T*) ssGetInputPortSignal(S,31);
	const real_T *linear_model32 = (const real_T*) ssGetInputPortSignal(S,32);
	const real_T *linear_model33 = (const real_T*) ssGetInputPortSignal(S,33);
	const real_T *linear_model34 = (const real_T*) ssGetInputPortSignal(S,34);
	const real_T *linear_model35 = (const real_T*) ssGetInputPortSignal(S,35);
	const real_T *linear_model36 = (const real_T*) ssGetInputPortSignal(S,36);
	const real_T *linear_model37 = (const real_T*) ssGetInputPortSignal(S,37);
	const real_T *linear_model38 = (const real_T*) ssGetInputPortSignal(S,38);
	const real_T *linear_model39 = (const real_T*) ssGetInputPortSignal(S,39);
	const real_T *f_error1 = (const real_T*) ssGetInputPortSignal(S,40);
	const real_T *f_error2 = (const real_T*) ssGetInputPortSignal(S,41);
	const real_T *f_error3 = (const real_T*) ssGetInputPortSignal(S,42);
	const real_T *f_error4 = (const real_T*) ssGetInputPortSignal(S,43);
	const real_T *f_error5 = (const real_T*) ssGetInputPortSignal(S,44);
	const real_T *f_error6 = (const real_T*) ssGetInputPortSignal(S,45);
	const real_T *f_error7 = (const real_T*) ssGetInputPortSignal(S,46);
	const real_T *f_error8 = (const real_T*) ssGetInputPortSignal(S,47);
	const real_T *f_error9 = (const real_T*) ssGetInputPortSignal(S,48);
	const real_T *f_error10 = (const real_T*) ssGetInputPortSignal(S,49);
	const real_T *f_error11 = (const real_T*) ssGetInputPortSignal(S,50);
	const real_T *f_error12 = (const real_T*) ssGetInputPortSignal(S,51);
	const real_T *f_error13 = (const real_T*) ssGetInputPortSignal(S,52);
	const real_T *f_error14 = (const real_T*) ssGetInputPortSignal(S,53);
	const real_T *f_error15 = (const real_T*) ssGetInputPortSignal(S,54);
	const real_T *f_error16 = (const real_T*) ssGetInputPortSignal(S,55);
	const real_T *f_error17 = (const real_T*) ssGetInputPortSignal(S,56);
	const real_T *f_error18 = (const real_T*) ssGetInputPortSignal(S,57);
	const real_T *f_error19 = (const real_T*) ssGetInputPortSignal(S,58);
	const real_T *f_error20 = (const real_T*) ssGetInputPortSignal(S,59);
	const real_T *f_error21 = (const real_T*) ssGetInputPortSignal(S,60);
	const real_T *f_error22 = (const real_T*) ssGetInputPortSignal(S,61);
	const real_T *f_error23 = (const real_T*) ssGetInputPortSignal(S,62);
	const real_T *f_error24 = (const real_T*) ssGetInputPortSignal(S,63);
	const real_T *f_error25 = (const real_T*) ssGetInputPortSignal(S,64);
	const real_T *f_error26 = (const real_T*) ssGetInputPortSignal(S,65);
	const real_T *f_error27 = (const real_T*) ssGetInputPortSignal(S,66);
	const real_T *f_error28 = (const real_T*) ssGetInputPortSignal(S,67);
	const real_T *f_error29 = (const real_T*) ssGetInputPortSignal(S,68);
	const real_T *f_error30 = (const real_T*) ssGetInputPortSignal(S,69);
	const real_T *f_error31 = (const real_T*) ssGetInputPortSignal(S,70);
	const real_T *f_error32 = (const real_T*) ssGetInputPortSignal(S,71);
	const real_T *f_error33 = (const real_T*) ssGetInputPortSignal(S,72);
	const real_T *f_error34 = (const real_T*) ssGetInputPortSignal(S,73);
	const real_T *f_error35 = (const real_T*) ssGetInputPortSignal(S,74);
	const real_T *f_error36 = (const real_T*) ssGetInputPortSignal(S,75);
	const real_T *f_error37 = (const real_T*) ssGetInputPortSignal(S,76);
	const real_T *f_error38 = (const real_T*) ssGetInputPortSignal(S,77);
	const real_T *f_error39 = (const real_T*) ssGetInputPortSignal(S,78);
	
    real_T *output = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	static MPC_Project_FORCESPRO_params params;
	static MPC_Project_FORCESPRO_output output;
	static MPC_Project_FORCESPRO_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<3; i++)
	{ 
		params.xinit[i] = (double) xinit[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model1[i] = (double) linear_model1[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model2[i] = (double) linear_model2[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model3[i] = (double) linear_model3[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model4[i] = (double) linear_model4[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model5[i] = (double) linear_model5[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model6[i] = (double) linear_model6[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model7[i] = (double) linear_model7[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model8[i] = (double) linear_model8[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model9[i] = (double) linear_model9[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model10[i] = (double) linear_model10[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model11[i] = (double) linear_model11[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model12[i] = (double) linear_model12[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model13[i] = (double) linear_model13[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model14[i] = (double) linear_model14[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model15[i] = (double) linear_model15[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model16[i] = (double) linear_model16[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model17[i] = (double) linear_model17[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model18[i] = (double) linear_model18[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model19[i] = (double) linear_model19[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model20[i] = (double) linear_model20[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model21[i] = (double) linear_model21[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model22[i] = (double) linear_model22[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model23[i] = (double) linear_model23[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model24[i] = (double) linear_model24[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model25[i] = (double) linear_model25[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model26[i] = (double) linear_model26[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model27[i] = (double) linear_model27[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model28[i] = (double) linear_model28[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model29[i] = (double) linear_model29[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model30[i] = (double) linear_model30[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model31[i] = (double) linear_model31[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model32[i] = (double) linear_model32[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model33[i] = (double) linear_model33[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model34[i] = (double) linear_model34[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model35[i] = (double) linear_model35[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model36[i] = (double) linear_model36[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model37[i] = (double) linear_model37[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model38[i] = (double) linear_model38[i]; 
	}

	for( i=0; i<15; i++)
	{ 
		params.linear_model39[i] = (double) linear_model39[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error1[i] = (double) f_error1[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error2[i] = (double) f_error2[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error3[i] = (double) f_error3[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error4[i] = (double) f_error4[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error5[i] = (double) f_error5[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error6[i] = (double) f_error6[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error7[i] = (double) f_error7[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error8[i] = (double) f_error8[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error9[i] = (double) f_error9[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error10[i] = (double) f_error10[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error11[i] = (double) f_error11[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error12[i] = (double) f_error12[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error13[i] = (double) f_error13[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error14[i] = (double) f_error14[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error15[i] = (double) f_error15[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error16[i] = (double) f_error16[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error17[i] = (double) f_error17[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error18[i] = (double) f_error18[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error19[i] = (double) f_error19[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error20[i] = (double) f_error20[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error21[i] = (double) f_error21[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error22[i] = (double) f_error22[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error23[i] = (double) f_error23[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error24[i] = (double) f_error24[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error25[i] = (double) f_error25[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error26[i] = (double) f_error26[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error27[i] = (double) f_error27[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error28[i] = (double) f_error28[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error29[i] = (double) f_error29[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error30[i] = (double) f_error30[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error31[i] = (double) f_error31[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error32[i] = (double) f_error32[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error33[i] = (double) f_error33[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error34[i] = (double) f_error34[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error35[i] = (double) f_error35[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error36[i] = (double) f_error36[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error37[i] = (double) f_error37[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error38[i] = (double) f_error38[i]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error39[i] = (double) f_error39[i]; 
	}

	

	

    #if SET_PRINTLEVEL_MPC_Project_FORCESPRO > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = MPC_Project_FORCESPRO_solve(&params, &output, &info, fp );

	#if SET_PRINTLEVEL_MPC_Project_FORCESPRO > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<50; i++)
	{ 
		output[i] = (real_T) output.output[i]; 
	}

	
}





/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


