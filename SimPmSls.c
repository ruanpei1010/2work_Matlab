/** @defgroup <Module_Name>
 *  @addtogroup <Module_Name>(If this is a sub-module, add it father module here)
 *  @design <Document link>
 *  @testspec <Document link>
 *
 *  @{
 */
//------------------------------------------------------------------------------
//  $Header$
//
//  Company    : HolyDrive Control Technologies, Co., Ltd.
//
//  Project    : P14103
//
//  Filename   : test.c
//
//  Programmer : 
//
//  Description   : S-FUNCTION MODULE
//
//              ***  Confidential property of HolyDrive Control Technologies ***
//                             Copyright(c) HolyDrive Control Technologies, 2014
//------------------------------------------------------------------------------
#define S_FUNCTION_NAME SimPmSls
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "PmCtrl.h"
#include "PmSls.h"
#include "DataType.h"
#include "MathAbs.h"
#include "MathConst.h"

//-----------------------------------------------------------------------
PM_CTRL_OUT     g_sOut;
PM_CTRL_PARAMS  g_sParams= PM_CTRL_PARAMS_DEFAULTS;
S_CTRL_FB     g_sFdb;

static void mdlInitializeSizes(SimStruct *S)
{
   // Set Parameter Number
   ssSetNumSFcnParams(S, 1);
   if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
   {
      return;
   }
   // set input port
   if (!ssSetNumInputPorts(S, 2)) return;
   // SpdRef
   ssSetInputPortWidth(S, 0, 1);
   ssSetInputPortDirectFeedThrough(S, 0, 1);
   // Feedback
   ssSetInputPortWidth(S, 1, 6);
   ssSetInputPortDirectFeedThrough(S, 1, 1);

   // set output ports
   if (!ssSetNumOutputPorts(S, 2)) return;
   // Cmp
   ssSetOutputPortWidth(S, 0, 3);
   // Test
   ssSetOutputPortWidth(S, 1, 4);

   ssSetNumSampleTimes(S, 1);
   ssSetNumNonsampledZCs(S, 0);
   ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
   FLOAT32 f32Tsw;
   f32Tsw = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_TSW]);
   ssSetSampleTime(S, 0, f32Tsw);
   ssSetOffsetTime(S, 0, 0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
   g_sParams.f32Tsw          = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_TSW]);
   g_sParams.f32Rs           = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_RS]);
   g_sParams.f32Ld           = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_LSD]);
   g_sParams.f32Lq           = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_LSQ]);
   g_sParams.f32BackEMF3kRpm = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_E3000RPM]); //E_rms at Nnom
   g_sParams.f32Inom         = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_INOM]); //Inom
   g_sParams.f32FoutNom      = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_FOUTNOM]);
   g_sParams.f32Np           = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_NP]);
   g_sParams.f32Nnom         = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_NNOM]); //Nnom
   g_sParams.f32Pnom         = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_PNOM]); //Pnom
   g_sParams.f32Imax         = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_CURMAX]);//Imax
   g_sParams.f32Nmax         = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_NMAX]);//Nmax
   g_sParams.f32DeadTime     = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_DEADTIME]);
   g_sParams.f32Unom         = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_UNOM]);//Unom
   g_sParams.f32ParkTime     = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_PART_TIME]);
   g_sParams.f32RampUp       = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_RAMPUP_T]);
   g_sParams.f32RampDown     = (FLOAT32)((mxGetPr(ssGetSFcnParam(S,0)))[E_RAMPDOWN_T]);

   PMSLS_Reset();
   PMSLS_Initial(&g_sParams);
   printf("f32Tsw=%f,f32Rs=%f,f32Ld=%f,f32Lq=%f\r\n", g_sParams.f32Tsw, g_sParams.f32Rs,
         g_sParams.f32Ld, g_sParams.f32Lq);
   printf("f32BackEMF3kRpm=%f,f32Inom=%f,f32FoutNom=%f,f32Np=%f\r\n", g_sParams.f32BackEMF3kRpm,
         g_sParams.f32Inom, g_sParams.f32FoutNom, g_sParams.f32Np);
   printf("f32Nnom=%f,f32Pnom=%f,f32Imax=%f,f32Nmax=%f\r\n", g_sParams.f32Nnom, g_sParams.f32Pnom,
         g_sParams.f32Imax, g_sParams.f32Nmax);
   printf("f32DeadTime=%f,f32Unom=%f,f32ParkTime=%f,f32RampUp=%f\r\n", g_sParams.f32DeadTime, g_sParams.f32Unom,
         g_sParams.f32ParkTime, g_sParams.f32RampUp);

}

static void mdlOutputs(SimStruct *S, int_T tid)
{
   InputRealPtrsType pInput;
   FLOAT64 *pOutput;
   FLOAT32 f32SpdRef;

   // In: Speed ref
   pInput = ssGetInputPortRealSignalPtrs(S,0);
   f32SpdRef = (FLOAT32)*pInput[0];

   // In:Feedback
   pInput = ssGetInputPortRealSignalPtrs(S,1);
   g_sFdb.f32Iu = (FLOAT32)*pInput[0];
   g_sFdb.f32Iv = (FLOAT32)*pInput[1];
   g_sFdb.f32Iw = (FLOAT32)*pInput[2];
   g_sFdb.f32Spd = 0.0f;//(FLOAT32)*pInput[3];
   g_sFdb.f32Theta = (FLOAT32)*pInput[4];
   g_sFdb.f32Udc = (FLOAT32)*pInput[5];

   // Calculation
   PMSLS_Cal(f32SpdRef, &g_sFdb, &g_sOut);

   // Out1: cmp
   pOutput = ssGetOutputPortRealSignal(S,0);
   pOutput[0] = (FLOAT64)(g_sOut.f32CmpU);
   pOutput[1] = (FLOAT64)(g_sOut.f32CmpV);
   pOutput[2] = (FLOAT64)(g_sOut.f32CmpW);

   pOutput = ssGetOutputPortRealSignal(S,1);
   pOutput[0] = (FLOAT64)(g_sOut.f32Test1);
   pOutput[1] = (FLOAT64)(g_sOut.f32Test2);
   pOutput[2] = (FLOAT64)(g_sOut.f32Test3);
   pOutput[3] = (FLOAT64)(g_sOut.f32Test4);

}

static void mdlTerminate(SimStruct *S)
{
   PMSLS_Reset();
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
