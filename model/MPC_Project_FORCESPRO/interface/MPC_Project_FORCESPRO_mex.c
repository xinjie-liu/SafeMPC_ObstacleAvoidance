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

#include "mex.h"
#include "math.h"
#include "../include/MPC_Project_FORCESPRO.h"
#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif

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

/* copy functions */

void copyCArrayToM_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double) (*src++) ;
    }
}

void copyMValueToC_double(double * src, double * dest)
{
	*dest = (double) *src;
}





/* Some memory for mex-function */
static MPC_Project_FORCESPRO_params params;
static MPC_Project_FORCESPRO_output output;
static MPC_Project_FORCESPRO_info info;

/* THE mex-function */
void mexFunction( solver_int32_default nlhs, mxArray *plhs[], solver_int32_default nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0]; 
	double *pvalue;
	solver_int32_default i;
	solver_int32_default exitflag;
	const solver_int8_default *fname;
	const solver_int8_default *outputnames[1] = {"output"};
	const solver_int8_default *infofields[16] = { "it", "it2opt", "res_eq", "res_ineq",  "pobj",  "dobj",  "dgap", "rdgap",  "mu",  "mu_aff",  "sigma",  "lsit_aff",  "lsit_cc",  "step_aff",   "step_cc",  "solvetime"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1)
	{
		mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help MPC_Project_FORCESPRO_mex' for details.");
	}    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help MPC_Project_FORCESPRO_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) 
	{
		mexErrMsgTxt("PARAMS must be a structure.");
	}
	 

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "xinit");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.xinit not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.xinit must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.xinit must be of size [3 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.xinit,3);

	}
	par = mxGetField(PARAMS, 0, "linear_model1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model1 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model1 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model1,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model2 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model2 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model2,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model3 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model3 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model3,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model4 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model4 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model4,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model5 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model5 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model5,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model6 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model6 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model6,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model7 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model7 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model7,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model8 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model8 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model8,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model9 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model9 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model9,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model10 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model10 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model10,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model11 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model11 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model11,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model12 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model12 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model12,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model13 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model13 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model13,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model14 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model14 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model14,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model15 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model15 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model15,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model16 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model16 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model16,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model17 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model17 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model17,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model18 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model18 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model18,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model19 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model19 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model19,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model20 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model20 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model20,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model21 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model21 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model21,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model22 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model22 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model22,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model23 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model23 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model23,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model24 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model24 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model24,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model25 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model25 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model25,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model26 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model26 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model26,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model27 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model27 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model27,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model28 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model28 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model28,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model29 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model29 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model29,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model30 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model30 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model30,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model31 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model31 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model31,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model32 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model32 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model32,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model33 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model33 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model33,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model34 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model34 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model34,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model35 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model35 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model35,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model36 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model36 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model36,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model37 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model37 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model37,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model38 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model38 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model38,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model39 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model39 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model39,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model40 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model40 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model40,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model41 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model41 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model41,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model42 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model42 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model42,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model43 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model43 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model43,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model44 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model44 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model44,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model45 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model45 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model45,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model46 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model46 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model46,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model47 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model47 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model47,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model48 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model48 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model48,15);

	}
	par = mxGetField(PARAMS, 0, "linear_model49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.linear_model49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.linear_model49 must be a double.");
    }
    if( mxGetM(par) != 3 || mxGetN(par) != 5 ) 
	{
    mexErrMsgTxt("PARAMS.linear_model49 must be of size [3 x 5]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.linear_model49,15);

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

	/* call solver */
	exitflag = MPC_Project_FORCESPRO_solve(&params, &output, &info, fp);
	
	#if SET_PRINTLEVEL_MPC_Project_FORCESPRO > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 1, outputnames);
		outvar = mxCreateDoubleMatrix(50, 1, mxREAL);
	copyCArrayToM_double( output.output, mxGetPr(outvar), 50);
	mxSetField(plhs[0], 0, "output", outvar);



	/* copy exitflag */
	if( nlhs > 1 )
	{
	plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
	*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
	        plhs[2] = mxCreateStructMatrix(1, 1, 16, infofields);
         
		
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* iterations to optimality (branch and bound) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it2opt;
		mxSetField(plhs[2], 0, "it2opt", outvar);
		
		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* res_ineq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_ineq;
		mxSetField(plhs[2], 0, "res_ineq", outvar);

		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* dobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dobj;
		mxSetField(plhs[2], 0, "dobj", outvar);

		/* dgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dgap;
		mxSetField(plhs[2], 0, "dgap", outvar);

		/* rdgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rdgap;
		mxSetField(plhs[2], 0, "rdgap", outvar);

		/* mu */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu;
		mxSetField(plhs[2], 0, "mu", outvar);

		/* mu_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu_aff;
		mxSetField(plhs[2], 0, "mu_aff", outvar);

		/* sigma */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.sigma;
		mxSetField(plhs[2], 0, "sigma", outvar);

		/* lsit_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_aff;
		mxSetField(plhs[2], 0, "lsit_aff", outvar);

		/* lsit_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_cc;
		mxSetField(plhs[2], 0, "lsit_cc", outvar);

		/* step_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_aff;
		mxSetField(plhs[2], 0, "step_aff", outvar);

		/* step_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_cc;
		mxSetField(plhs[2], 0, "step_cc", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);

	}
}
