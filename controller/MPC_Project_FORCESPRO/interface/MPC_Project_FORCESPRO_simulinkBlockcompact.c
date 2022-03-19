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
#define S_FUNCTION_NAME MPC_Project_FORCESPRO_simulinkBlockcompact

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

	/* initialize input ports - there are 4 in total */
    if (!ssSetNumInputPorts(S, 4)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 6, 90);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 1, 10);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 1, 100);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 6, 1);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/
 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 100, 1);
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
	const real_T *C = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *b = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *A = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *xinit = (const real_T*) ssGetInputPortSignal(S,3);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	static MPC_Project_FORCESPRO_params params;
	static MPC_Project_FORCESPRO_output output;
	static MPC_Project_FORCESPRO_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<6; i++)
	{ 
		params.xinit[i] = (double) xinit[i]; 
	}

	for( i=0; i<60; i++)
	{ 
		params.linear_model1[i] = (double) C[i]; 
	}

	j=60; 
	for( i=0; i<60; i++)
	{ 
		params.linear_model2[i] = (double) C[j++]; 
	}

	j=120; 
	for( i=0; i<60; i++)
	{ 
		params.linear_model3[i] = (double) C[j++]; 
	}

	j=180; 
	for( i=0; i<60; i++)
	{ 
		params.linear_model4[i] = (double) C[j++]; 
	}

	j=240; 
	for( i=0; i<60; i++)
	{ 
		params.linear_model5[i] = (double) C[j++]; 
	}

	j=300; 
	for( i=0; i<60; i++)
	{ 
		params.linear_model6[i] = (double) C[j++]; 
	}

	j=360; 
	for( i=0; i<60; i++)
	{ 
		params.linear_model7[i] = (double) C[j++]; 
	}

	j=420; 
	for( i=0; i<60; i++)
	{ 
		params.linear_model8[i] = (double) C[j++]; 
	}

	j=480; 
	for( i=0; i<60; i++)
	{ 
		params.linear_model9[i] = (double) C[j++]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA1[i] = (double) A[i]; 
	}

	j=10; 
	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA2[i] = (double) A[j++]; 
	}

	j=20; 
	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA3[i] = (double) A[j++]; 
	}

	j=30; 
	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA4[i] = (double) A[j++]; 
	}

	j=40; 
	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA5[i] = (double) A[j++]; 
	}

	j=50; 
	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA6[i] = (double) A[j++]; 
	}

	j=60; 
	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA7[i] = (double) A[j++]; 
	}

	j=70; 
	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA8[i] = (double) A[j++]; 
	}

	j=80; 
	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA9[i] = (double) A[j++]; 
	}

	j=90; 
	for( i=0; i<10; i++)
	{ 
		params.hyperplaneA10[i] = (double) A[j++]; 
	}

	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb1[i] = (double) b[i]; 
	}

	j=1; 
	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb2[i] = (double) b[j++]; 
	}

	j=2; 
	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb3[i] = (double) b[j++]; 
	}

	j=3; 
	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb4[i] = (double) b[j++]; 
	}

	j=4; 
	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb5[i] = (double) b[j++]; 
	}

	j=5; 
	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb6[i] = (double) b[j++]; 
	}

	j=6; 
	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb7[i] = (double) b[j++]; 
	}

	j=7; 
	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb8[i] = (double) b[j++]; 
	}

	j=8; 
	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb9[i] = (double) b[j++]; 
	}

	j=9; 
	for( i=0; i<1; i++)
	{ 
		params.hyperplaneb10[i] = (double) b[j++]; 
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
	for( i=0; i<100; i++)
	{ 
		outputs[i] = (real_T) output.output[i]; 
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


