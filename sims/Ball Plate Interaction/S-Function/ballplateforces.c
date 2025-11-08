/* Ball Plate Forces
 * 
 * 
 */ 

#define S_FUNCTION_NAME  	ballplateforces
#define S_FUNCTION_LEVEL	2
#include "simstruc.h"
#include <math.h>
#define PAR(element)		(*mxGetPr(ssGetSFcnParam(S,element)))
#define U(element) 			(*uPtrs[element])

/* Parameters. */
#define xpla	 			PAR(0)	/* Plane length [m]. */
#define ypla	 			PAR(1)	/* Plane width [m]. */
#define zpla	 			PAR(2)	/* Plane thickness [m]. */
#define rbal	 			PAR(3)	/* Ball radius [m]. */
#define Kpen	 			PAR(4)	/* Spring constant [N/m]. */
#define Dpen	 			PAR(5)	/* Damping constant [N.s/m]. */
#define mustat	 			PAR(6)	/* Static friction constant [ ]. */
#define vthr	 			PAR(7)	/* Friction threshold speed [m/s]. */

/* Inputs. */
#define xpos                U(0)    /* Ball position in x-direction. */
#define vx                  U(1)    /* Ball speed in the x direction. [m/s]  */
#define ypos                U(2)    /* Ball position in y-direction. */
#define vy                  U(3)	/* Ball speed in the y direction. [m/s] */
#define pz                  U(4)	/* Ball position in the z direction. [m] */
#define vz                  U(5)    /* Ball speed in the z direction. [m/s] */
#define wx                  U(6)	/* Ball rotation speed around the x axis. [rad/s] */
#define wy                  U(7)	/* Ball rotation speed around the y axis. [rad/s] */
#define Tsim                U(8)	/* Simulation time. [s] */






/* Initialization. */
static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, 8);  
   if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 9);
    ssSetInputPortDirectFeedThrough(S, 0, 9);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 5);

    ssSetNumSampleTimes(S, 1);
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */

static void mdlOutputs(SimStruct *S, int_T tid) {
	real_T              *y    = ssGetOutputPortRealSignal(S,0);
	real_T              *x    = ssGetContStates(S);	
    InputRealPtrsType   uPtrs = ssGetInputPortRealSignalPtrs(S,0);	        
    real_T              cgap, Fz;
    real_T              vxslippage, vyslippage;
    real_T              mux, muy, Ffx, Ffy;
    real_T              Tx, Ty;
    
    /* Support force parameters. */    
    cgap = zpla/2+rbal;
        
    /* Support force Fz (eq 1) when the ball is on the plate, and simulation time is post initialization . */
    if (pz<cgap && fabs(xpos)<xpla/2 && fabs(ypos)<ypla/2 && Tsim>0.001) 
        Fz = -Kpen*(pz-cgap)-Dpen*vz;
    else
        Fz = 0;
        
    /* Slippage speed(eq 6 and 7). */
    vxslippage = -vx+rbal*wy;
    vyslippage = -vy-rbal*wx;
    
    
    if (fabs(vxslippage) <= vthr)
     /* Friction coefficient in the x direction(eq 4). */
        mux = mustat * vxslippage/vthr;
    else if(vxslippage > vthr)
     /* Friction coefficient in the x direction for positive velocity. */
        mux = mustat;
    else
    /* Friction coefficient in the x direction for positive velocity. */
        mux = -mustat;    
    
    /* Friction force in the x direction(eq 2). */
    Ffx = mux*Fz;
   
    
    if (fabs(vyslippage) <= vthr)
     /* Friction coefficient in the y direction(eq 5). */
        muy = mustat * vyslippage/vthr;
    else if(vyslippage > vthr)
     /* Friction coefficient in the y direction for positive velocity. */
        muy = mustat;
    else
    /* Friction coefficient in the y direction for positive velocity. */
        muy = -mustat;    
    
    /* Friction force in the y direction(eq 3). */
    Ffy = muy*Fz;
    
    
    /* Rolling torque around the x axis(eq 8). */
    Tx = rbal*Ffy;
    
   /* Rolling torque around the x axis(eq 9). */
    Ty = -rbal*Ffx;
    
    y[0] = Ffx; 
	y[1] = Ffy;
    y[2] = Fz; 
	y[3] = Tx;
    y[4] = Ty;
    
    
   
}

#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
