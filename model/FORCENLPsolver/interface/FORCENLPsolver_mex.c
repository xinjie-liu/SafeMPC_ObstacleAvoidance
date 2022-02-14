/*
FORCENLPsolver : A fast customized optimization solver.

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
#include "../include/FORCENLPsolver.h"
#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
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

/* copy functions */

void copyCArrayToM_FORCENLPsolver_int(FORCENLPsolver_int *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_FORCENLPsolver_int(double *src, FORCENLPsolver_int *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (FORCENLPsolver_int) (*src++) ;
    }
}

void copyMValueToC_FORCENLPsolver_int(double * src, FORCENLPsolver_int * dest)
{
	*dest = (FORCENLPsolver_int) *src;
}



extern void (FORCENLPsolver_float *x, FORCENLPsolver_float *y, FORCENLPsolver_float *l, FORCENLPsolver_float *p, FORCENLPsolver_float *f, FORCENLPsolver_float *nabla_f, FORCENLPsolver_float *c, FORCENLPsolver_float *nabla_c, FORCENLPsolver_float *h, FORCENLPsolver_float *nabla_h, FORCENLPsolver_float *hess, solver_int32_default stage, solver_int32_default iteration, solver_int32_default threadID);
FORCENLPsolver_extfunc pt2function_FORCENLPsolver = &;


/* Some memory for mex-function */
static FORCENLPsolver_params params;
static FORCENLPsolver_output output;
static FORCENLPsolver_info info;

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
	const solver_int8_default *outputnames[40] = {"x01","x02","x03","x04","x05","x06","x07","x08","x09","x10","x11","x12","x13","x14","x15","x16","x17","x18","x19","x20","x21","x22","x23","x24","x25","x26","x27","x28","x29","x30","x31","x32","x33","x34","x35","x36","x37","x38","x39","x40"};
	const solver_int8_default *infofields[7] = { "it", "res_eq", "rsnorm", "pobj", "solvetime",  "fevalstime", "QPtime"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1)
	{
		mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help FORCENLPsolver_mex' for details.");
	}    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help FORCENLPsolver_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) 
	{
		mexErrMsgTxt("PARAMS must be a structure.");
	}
	 

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "lb");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.lb not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb must be a double.");
    }
    if( mxGetM(par) != 160 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.lb must be of size [160 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.lb,160);

	}
	par = mxGetField(PARAMS, 0, "ub");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.ub not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub must be a double.");
    }
    if( mxGetM(par) != 160 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.ub must be of size [160 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.ub,160);

	}
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
    if( mxGetM(par) != 13 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.xinit must be of size [13 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.xinit,13);

	}
	par = mxGetField(PARAMS, 0, "x0");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.x0 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.x0 must be a double.");
    }
    if( mxGetM(par) != 680 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.x0 must be of size [680 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.x0,680);

	}
	par = mxGetField(PARAMS, 0, "all_parameters");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.all_parameters not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.all_parameters must be a double.");
    }
    if( mxGetM(par) != 360 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.all_parameters must be of size [360 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.all_parameters,360);

	}
	par = mxGetField(PARAMS, 0, "reinitialize");
	if ( (par != NULL) && (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMValueToC_FORCENLPsolver_int(mxGetPr(par), &params.reinitialize);

	}




	#if SET_PRINTLEVEL_FORCENLPsolver > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* call solver */
	exitflag = FORCENLPsolver_solve(&params, &output, &info, fp, pt2function_FORCENLPsolver);
	
	#if SET_PRINTLEVEL_FORCENLPsolver > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 40, outputnames);
		outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x01, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x01", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x02, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x02", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x03, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x03", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x04, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x04", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x05, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x05", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x06, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x06", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x07, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x07", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x08, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x08", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x09, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x09", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x10, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x10", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x11, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x11", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x12, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x12", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x13, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x13", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x14, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x14", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x15, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x15", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x16, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x16", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x17, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x17", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x18, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x18", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x19, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x19", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x20, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x20", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x21, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x21", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x22, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x22", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x23, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x23", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x24, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x24", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x25, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x25", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x26, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x26", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x27, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x27", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x28, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x28", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x29, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x29", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x30, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x30", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x31, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x31", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x32, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x32", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x33, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x33", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x34, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x34", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x35, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x35", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x36, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x36", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x37, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x37", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x38, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x38", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x39, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x39", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM_double( output.x40, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "x40", outvar);



	/* copy exitflag */
	if( nlhs > 1 )
	{
	plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
	*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
	        plhs[2] = mxCreateStructMatrix(1, 1, 7, infofields);
        
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* rsnorm */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rsnorm;
		mxSetField(plhs[2], 0, "rsnorm", outvar);

		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);

		/* fevals time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.fevalstime;
		mxSetField(plhs[2], 0, "fevalstime", outvar);
		
		/* QP time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.QPtime;
		mxSetField(plhs[2], 0, "QPtime", outvar);		

	}
}
