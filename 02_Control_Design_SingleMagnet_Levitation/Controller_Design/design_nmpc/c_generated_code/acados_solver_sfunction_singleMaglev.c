/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#define S_FUNCTION_NAME acados_solver_sfunction_singleMaglev
#define S_FUNCTION_LEVEL 2

#define MDL_START

// acados
// #include "acados/utils/print.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "acados_solver_singleMaglev.h"



#include "simstruc.h"

#define SAMPLINGTIME -1


  
  
  
  
  
  
  
  
  
  
  
  
  
  



static void mdlInitializeSizes (SimStruct *S)
{
    // specify the number of continuous and discrete states
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    int N = 20;// Additional input for payload (only when enabled)

  

  

  

  

  

  

  

    // specify the number of input ports
    if ( !ssSetNumInputPorts(S, 15) )
        return;

    // specify the number of output ports
    if ( !ssSetNumOutputPorts(S, 9) )
        return;

    // specify dimension information for the input ports
    // lbx_0
    ssSetInputPortVectorDimension(S, 0, 2);
    // ubx_0
    ssSetInputPortVectorDimension(S, 1, 2);
    // parameter_traj
    ssSetInputPortVectorDimension(S, 2, 84);
    // y_ref_0
    ssSetInputPortVectorDimension(S, 3, 3);


    // y_ref
    ssSetInputPortVectorDimension(S, 4, 57);
    // y_ref_e
    ssSetInputPortVectorDimension(S, 5, 2);
    // lh
    ssSetInputPortVectorDimension(S, 6, 19);
    // uh
    ssSetInputPortVectorDimension(S, 7, 19);
    // lh_0
    ssSetInputPortVectorDimension(S, 8, 1);
    // uh_0
    ssSetInputPortVectorDimension(S, 9, 1);
    // cost_W_0
    ssSetInputPortVectorDimension(S, 10, 9);
    // cost_W
    ssSetInputPortVectorDimension(S, 11, 9);
    // cost_W_e
    ssSetInputPortVectorDimension(S, 12, 4);
    // reset_solver
    ssSetInputPortVectorDimension(S, 13, 1);
    // x_init
    ssSetInputPortVectorDimension(S, 14, 42);/* specify dimension information for the OUTPUT ports */
    ssSetOutputPortVectorDimension(S, 0, 1 );
    ssSetOutputPortVectorDimension(S, 1, 20 );
    ssSetOutputPortVectorDimension(S, 2, 42 );
    ssSetOutputPortVectorDimension(S, 3, 1 );
    ssSetOutputPortVectorDimension(S, 4, 1 );
    ssSetOutputPortVectorDimension(S, 5, 4 );
    ssSetOutputPortVectorDimension(S, 6, 2 ); // state at shooting node 1
    ssSetOutputPortVectorDimension(S, 7, 1);
    ssSetOutputPortVectorDimension(S, 8, 1 );
    // specify the direct feedthrough status
    // should be set to 1 for all inputs used in mdlOutputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 6, 1);
    ssSetInputPortDirectFeedThrough(S, 7, 1);
    ssSetInputPortDirectFeedThrough(S, 8, 1);
    ssSetInputPortDirectFeedThrough(S, 9, 1);
    ssSetInputPortDirectFeedThrough(S, 10, 1);
    ssSetInputPortDirectFeedThrough(S, 11, 1);
    ssSetInputPortDirectFeedThrough(S, 12, 1);
    ssSetInputPortDirectFeedThrough(S, 13, 1);
    ssSetInputPortDirectFeedThrough(S, 14, 1);


    // one sample time
    ssSetNumSampleTimes(S, 1);
}


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

#endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    singleMaglev_solver_capsule *capsule = singleMaglev_acados_create_capsule();
    singleMaglev_acados_create(capsule);

    ssSetUserData(S, (void*)capsule);
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
    singleMaglev_solver_capsule *capsule = ssGetUserData(S);
    ocp_nlp_config *nlp_config = singleMaglev_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = singleMaglev_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = singleMaglev_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = singleMaglev_acados_get_nlp_out(capsule);
    ocp_nlp_solver *nlp_solver = singleMaglev_acados_get_nlp_solver(capsule);

    InputRealPtrsType in_sign;

    int N = 20;  

    

    // local buffer
    double buffer[84];
    double tmp_double;
    int tmp_offset, tmp_int;

    /* go through inputs */
    // lbx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 0);
    for (int i = 0; i < 2; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", buffer);
    // ubx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 1);
    for (int i = 0; i < 2; i++)
        buffer[i] = (double)(*in_sign[i]);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", buffer);
    // parameter_traj
    in_sign = ssGetInputPortRealSignalPtrs(S, 2);
    // update value of parameters
    tmp_offset = 0;
    for (int stage = 0; stage <= N; stage++)
    {
        tmp_int = ocp_nlp_dims_get_from_attr(nlp_config, nlp_dims, nlp_out, stage, "p");
        for (int jj = 0; jj < tmp_int; jj++)
        {
            buffer[jj] = (double)(*in_sign[tmp_offset+jj]);
        }
        singleMaglev_acados_update_params(capsule, stage, buffer, tmp_int);
        tmp_offset += tmp_int;
    }

  
    // y_ref_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 3);

    for (int i = 0; i < 3; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", (void *) buffer);


  
    // y_ref - for stages 1 to N-1
    in_sign = ssGetInputPortRealSignalPtrs(S, 4);

    for (int stage = 1; stage < N; stage++)
    {
        for (int jj = 0; jj < 3; jj++)
            buffer[jj] = (double)(*in_sign[(stage-1)*3+jj]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", (void *) buffer);
    }

  
    // y_ref_e
    in_sign = ssGetInputPortRealSignalPtrs(S, 5);

    for (int i = 0; i < 2; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", (void *) buffer);
    // lh
    in_sign = ssGetInputPortRealSignalPtrs(S, 6);
    tmp_offset = 0;
    for (int stage = 1; stage < N; stage++)
    {
        tmp_int = ocp_nlp_dims_get_from_attr(nlp_config, nlp_dims, nlp_out, stage, "lh");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[tmp_offset+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, stage, "lh", (void *) buffer);
        tmp_offset += tmp_int;
    }
    // uh
    in_sign = ssGetInputPortRealSignalPtrs(S, 7);
    tmp_offset = 0;
    for (int stage = 1; stage < N; stage++)
    {
        tmp_int = ocp_nlp_dims_get_from_attr(nlp_config, nlp_dims, nlp_out, stage, "uh");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[tmp_offset+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, stage, "uh", (void *) buffer);
        tmp_offset += tmp_int;
    }
    // lh_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 8);
    for (int i = 0; i < 1; i++)
        buffer[i] = (double)(*in_sign[i]);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lh", buffer);
    // uh_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 9);
    for (int i = 0; i < 1; i++)
        buffer[i] = (double)(*in_sign[i]);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "uh", buffer);
    // cost_W_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 10);
    for (int i = 0; i < 9; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", buffer);
    // cost_W
    in_sign = ssGetInputPortRealSignalPtrs(S, 11);
    for (int i = 0; i < 9; i++)
        buffer[i] = (double)(*in_sign[i]);

    for (int stage = 1; stage < N; stage++)
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "W", buffer);
    // cost_W_e
    in_sign = ssGetInputPortRealSignalPtrs(S, 12);
    for (int i = 0; i < 4; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", buffer);
    // reset_solver
    in_sign = ssGetInputPortRealSignalPtrs(S, 13);
    double reset = (double)(*in_sign[0]);
    if (reset)
    {
        singleMaglev_acados_reset(capsule, 1);
    }

    int ignore_inits = 0;
    // ssPrintf("ignore_inits = %d\n", ignore_inits);

    if (ignore_inits == 0)
    {
        // x_init
        in_sign = ssGetInputPortRealSignalPtrs(S, 14);
        tmp_int = ocp_nlp_dims_get_total_from_attr(nlp_config, nlp_dims, nlp_out, "x");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[jj]);
        ocp_nlp_set_all(nlp_solver, nlp_in, nlp_out, "x", (void *) buffer);
    }/* call solver */
    int acados_status = singleMaglev_acados_solve(capsule);
    // get time
    ocp_nlp_get(nlp_solver, "time_tot", (void *) buffer);
    tmp_double = buffer[0];

    /* set outputs */
    double *out_ptr;
    out_ptr = ssGetOutputPortRealSignal(S, 0);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *) out_ptr);
    out_ptr = ssGetOutputPortRealSignal(S, 1);
    ocp_nlp_get_all(nlp_solver, nlp_in, nlp_out, "u", out_ptr);

  
    out_ptr = ssGetOutputPortRealSignal(S, 2);
    ocp_nlp_get_all(nlp_solver, nlp_in, nlp_out, "x", out_ptr);

  

  

  
    out_ptr = ssGetOutputPortRealSignal(S, 3);
    *out_ptr = (double) acados_status;
    out_ptr = ssGetOutputPortRealSignal(S, 4);
    ocp_nlp_eval_cost(nlp_solver, nlp_in, nlp_out);
    ocp_nlp_get(nlp_solver, "cost_value", (void *) out_ptr);
    out_ptr = ssGetOutputPortRealSignal(S, 5);
    ocp_nlp_get(nlp_solver, "res_stat", (void *) &out_ptr[0]);
    ocp_nlp_get(nlp_solver, "res_eq", (void *) &out_ptr[1]);
    ocp_nlp_get(nlp_solver, "res_ineq", (void *) &out_ptr[2]);
    ocp_nlp_get(nlp_solver, "res_comp", (void *) &out_ptr[3]);
    out_ptr = ssGetOutputPortRealSignal(S, 6);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *) out_ptr);
    out_ptr = ssGetOutputPortRealSignal(S, 7);
    out_ptr[0] = tmp_double;
    out_ptr = ssGetOutputPortRealSignal(S, 8);
    // get sqp iter
    ocp_nlp_get(nlp_solver, "sqp_iter", (void *) &tmp_int);
    *out_ptr = (double) tmp_int;

  

  

  

}

static void mdlTerminate(SimStruct *S)
{
    singleMaglev_solver_capsule *capsule = ssGetUserData(S);

    singleMaglev_acados_free(capsule);
    singleMaglev_acados_free_capsule(capsule);
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
