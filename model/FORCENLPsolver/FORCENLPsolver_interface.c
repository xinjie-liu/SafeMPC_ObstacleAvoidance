/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/FORCENLPsolver.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "FORCENLPsolver_model.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FORCENLPsolver_callback_float *data, FORCENLPsolver_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FORCENLPsolver_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void FORCENLPsolver_casadi2forces(FORCENLPsolver_float *x,        /* primal vars                                         */
                                 FORCENLPsolver_float *y,        /* eq. constraint multiplers                           */
                                 FORCENLPsolver_float *l,        /* ineq. constraint multipliers                        */
                                 FORCENLPsolver_float *p,        /* parameters                                          */
                                 FORCENLPsolver_float *f,        /* objective function (scalar)                         */
                                 FORCENLPsolver_float *nabla_f,  /* gradient of objective function                      */
                                 FORCENLPsolver_float *c,        /* dynamics                                            */
                                 FORCENLPsolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCENLPsolver_float *h,        /* inequality constraints                              */
                                 FORCENLPsolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCENLPsolver_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const FORCENLPsolver_callback_float *in[4];
    FORCENLPsolver_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	FORCENLPsolver_float w[31];
	
    /* temporary storage for CasADi sparse output */
    FORCENLPsolver_callback_float this_f;
    FORCENLPsolver_float nabla_f_sparse[5];
    
    
    FORCENLPsolver_float c_sparse[3];
    FORCENLPsolver_float nabla_c_sparse[10];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 <= stage && stage <= 18))
	{
		
		
		if( &this_f )
		{
			out[0] = &this_f;
			FORCENLPsolver_objective_0(in, out, NULL, w, 0);
		}
		
		if( nabla_f )
		{
			out[0] = nabla_f_sparse;
			FORCENLPsolver_dobjective_0(in, out, NULL, w, 0);
			nrow = FORCENLPsolver_dobjective_0_sparsity_out(0)[0];
			ncol = FORCENLPsolver_dobjective_0_sparsity_out(0)[1];
			colind = FORCENLPsolver_dobjective_0_sparsity_out(0) + 2;
			row = FORCENLPsolver_dobjective_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		if( c )
		{
			out[0] = c_sparse;
			FORCENLPsolver_dynamics_0(in, out, NULL, w, 0);
			nrow = FORCENLPsolver_dynamics_0_sparsity_out(0)[0];
			ncol = FORCENLPsolver_dynamics_0_sparsity_out(0)[1];
			colind = FORCENLPsolver_dynamics_0_sparsity_out(0) + 2;
			row = FORCENLPsolver_dynamics_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}
		
		if( nabla_c )
		{
			out[0] = nabla_c_sparse;
			FORCENLPsolver_ddynamics_0(in, out, NULL, w, 0);
			nrow = FORCENLPsolver_ddynamics_0_sparsity_out(0)[0];
			ncol = FORCENLPsolver_ddynamics_0_sparsity_out(0)[1];
			colind = FORCENLPsolver_ddynamics_0_sparsity_out(0) + 2;
			row = FORCENLPsolver_ddynamics_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}
	}
	if ((19 == stage))
	{
		
		
		if( &this_f )
		{
			out[0] = &this_f;
			FORCENLPsolver_objective_1(in, out, NULL, w, 0);
		}
		
		if( nabla_f )
		{
			out[0] = nabla_f_sparse;
			FORCENLPsolver_dobjective_1(in, out, NULL, w, 0);
			nrow = FORCENLPsolver_dobjective_1_sparsity_out(0)[0];
			ncol = FORCENLPsolver_dobjective_1_sparsity_out(0)[1];
			colind = FORCENLPsolver_dobjective_1_sparsity_out(0) + 2;
			row = FORCENLPsolver_dobjective_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FORCENLPsolver_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
