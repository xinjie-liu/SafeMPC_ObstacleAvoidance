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

	/* initialize input ports - there are 3 in total */
    if (!ssSetNumInputPorts(S, 3)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 3, 195);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 5, 39);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 3, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/
 


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
	const real_T *C = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *f = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *xinit = (const real_T*) ssGetInputPortSignal(S,2);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

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
		params.linear_model1[i] = (double) C[i]; 
	}

	j=15; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model2[i] = (double) C[j++]; 
	}

	j=30; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model3[i] = (double) C[j++]; 
	}

	j=45; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model4[i] = (double) C[j++]; 
	}

	j=60; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model5[i] = (double) C[j++]; 
	}

	j=75; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model6[i] = (double) C[j++]; 
	}

	j=90; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model7[i] = (double) C[j++]; 
	}

	j=105; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model8[i] = (double) C[j++]; 
	}

	j=120; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model9[i] = (double) C[j++]; 
	}

	j=135; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model10[i] = (double) C[j++]; 
	}

	j=150; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model11[i] = (double) C[j++]; 
	}

	j=165; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model12[i] = (double) C[j++]; 
	}

	j=180; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model13[i] = (double) C[j++]; 
	}

	j=195; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model14[i] = (double) C[j++]; 
	}

	j=210; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model15[i] = (double) C[j++]; 
	}

	j=225; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model16[i] = (double) C[j++]; 
	}

	j=240; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model17[i] = (double) C[j++]; 
	}

	j=255; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model18[i] = (double) C[j++]; 
	}

	j=270; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model19[i] = (double) C[j++]; 
	}

	j=285; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model20[i] = (double) C[j++]; 
	}

	j=300; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model21[i] = (double) C[j++]; 
	}

	j=315; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model22[i] = (double) C[j++]; 
	}

	j=330; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model23[i] = (double) C[j++]; 
	}

	j=345; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model24[i] = (double) C[j++]; 
	}

	j=360; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model25[i] = (double) C[j++]; 
	}

	j=375; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model26[i] = (double) C[j++]; 
	}

	j=390; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model27[i] = (double) C[j++]; 
	}

	j=405; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model28[i] = (double) C[j++]; 
	}

	j=420; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model29[i] = (double) C[j++]; 
	}

	j=435; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model30[i] = (double) C[j++]; 
	}

	j=450; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model31[i] = (double) C[j++]; 
	}

	j=465; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model32[i] = (double) C[j++]; 
	}

	j=480; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model33[i] = (double) C[j++]; 
	}

	j=495; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model34[i] = (double) C[j++]; 
	}

	j=510; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model35[i] = (double) C[j++]; 
	}

	j=525; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model36[i] = (double) C[j++]; 
	}

	j=540; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model37[i] = (double) C[j++]; 
	}

	j=555; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model38[i] = (double) C[j++]; 
	}

	j=570; 
	for( i=0; i<15; i++)
	{ 
		params.linear_model39[i] = (double) C[j++]; 
	}

	for( i=0; i<5; i++)
	{ 
		params.f_error1[i] = (double) f[i]; 
	}

	j=5; 
	for( i=0; i<5; i++)
	{ 
		params.f_error2[i] = (double) f[j++]; 
	}

	j=10; 
	for( i=0; i<5; i++)
	{ 
		params.f_error3[i] = (double) f[j++]; 
	}

	j=15; 
	for( i=0; i<5; i++)
	{ 
		params.f_error4[i] = (double) f[j++]; 
	}

	j=20; 
	for( i=0; i<5; i++)
	{ 
		params.f_error5[i] = (double) f[j++]; 
	}

	j=25; 
	for( i=0; i<5; i++)
	{ 
		params.f_error6[i] = (double) f[j++]; 
	}

	j=30; 
	for( i=0; i<5; i++)
	{ 
		params.f_error7[i] = (double) f[j++]; 
	}

	j=35; 
	for( i=0; i<5; i++)
	{ 
		params.f_error8[i] = (double) f[j++]; 
	}

	j=40; 
	for( i=0; i<5; i++)
	{ 
		params.f_error9[i] = (double) f[j++]; 
	}

	j=45; 
	for( i=0; i<5; i++)
	{ 
		params.f_error10[i] = (double) f[j++]; 
	}

	j=50; 
	for( i=0; i<5; i++)
	{ 
		params.f_error11[i] = (double) f[j++]; 
	}

	j=55; 
	for( i=0; i<5; i++)
	{ 
		params.f_error12[i] = (double) f[j++]; 
	}

	j=60; 
	for( i=0; i<5; i++)
	{ 
		params.f_error13[i] = (double) f[j++]; 
	}

	j=65; 
	for( i=0; i<5; i++)
	{ 
		params.f_error14[i] = (double) f[j++]; 
	}

	j=70; 
	for( i=0; i<5; i++)
	{ 
		params.f_error15[i] = (double) f[j++]; 
	}

	j=75; 
	for( i=0; i<5; i++)
	{ 
		params.f_error16[i] = (double) f[j++]; 
	}

	j=80; 
	for( i=0; i<5; i++)
	{ 
		params.f_error17[i] = (double) f[j++]; 
	}

	j=85; 
	for( i=0; i<5; i++)
	{ 
		params.f_error18[i] = (double) f[j++]; 
	}

	j=90; 
	for( i=0; i<5; i++)
	{ 
		params.f_error19[i] = (double) f[j++]; 
	}

	j=95; 
	for( i=0; i<5; i++)
	{ 
		params.f_error20[i] = (double) f[j++]; 
	}

	j=100; 
	for( i=0; i<5; i++)
	{ 
		params.f_error21[i] = (double) f[j++]; 
	}

	j=105; 
	for( i=0; i<5; i++)
	{ 
		params.f_error22[i] = (double) f[j++]; 
	}

	j=110; 
	for( i=0; i<5; i++)
	{ 
		params.f_error23[i] = (double) f[j++]; 
	}

	j=115; 
	for( i=0; i<5; i++)
	{ 
		params.f_error24[i] = (double) f[j++]; 
	}

	j=120; 
	for( i=0; i<5; i++)
	{ 
		params.f_error25[i] = (double) f[j++]; 
	}

	j=125; 
	for( i=0; i<5; i++)
	{ 
		params.f_error26[i] = (double) f[j++]; 
	}

	j=130; 
	for( i=0; i<5; i++)
	{ 
		params.f_error27[i] = (double) f[j++]; 
	}

	j=135; 
	for( i=0; i<5; i++)
	{ 
		params.f_error28[i] = (double) f[j++]; 
	}

	j=140; 
	for( i=0; i<5; i++)
	{ 
		params.f_error29[i] = (double) f[j++]; 
	}

	j=145; 
	for( i=0; i<5; i++)
	{ 
		params.f_error30[i] = (double) f[j++]; 
	}

	j=150; 
	for( i=0; i<5; i++)
	{ 
		params.f_error31[i] = (double) f[j++]; 
	}

	j=155; 
	for( i=0; i<5; i++)
	{ 
		params.f_error32[i] = (double) f[j++]; 
	}

	j=160; 
	for( i=0; i<5; i++)
	{ 
		params.f_error33[i] = (double) f[j++]; 
	}

	j=165; 
	for( i=0; i<5; i++)
	{ 
		params.f_error34[i] = (double) f[j++]; 
	}

	j=170; 
	for( i=0; i<5; i++)
	{ 
		params.f_error35[i] = (double) f[j++]; 
	}

	j=175; 
	for( i=0; i<5; i++)
	{ 
		params.f_error36[i] = (double) f[j++]; 
	}

	j=180; 
	for( i=0; i<5; i++)
	{ 
		params.f_error37[i] = (double) f[j++]; 
	}

	j=185; 
	for( i=0; i<5; i++)
	{ 
		params.f_error38[i] = (double) f[j++]; 
	}

	j=190; 
	for( i=0; i<5; i++)
	{ 
		params.f_error39[i] = (double) f[j++]; 
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


