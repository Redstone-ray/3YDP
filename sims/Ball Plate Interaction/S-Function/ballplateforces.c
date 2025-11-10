/* Ball Plate Forces — circular disk version
 *
 * Contact and friction model for a ball on a horizontal circular plate.
 * Key change vs. rectangular plate: in-plane support check uses radius.
 *
 * Inputs  (single port, width = 9):
 *   0 xpos  (m)   ball COM x
 *   1 vx    (m/s) ball COM x-velocity
 *   2 ypos  (m)   ball COM y
 *   3 vy    (m/s) ball COM y-velocity
 *   4 pz    (m)   ball COM z
 *   5 vz    (m/s) ball COM z-velocity
 *   6 wx    (rad/s) ball angular velocity about x
 *   7 wy    (rad/s) ball angular velocity about y
 *   8 Tsim  (s)   simulation time
 *
 * Outputs (single port, width = 5):
 *   0 Ffx (N)  friction force x
 *   1 Ffy (N)  friction force y
 *   2 Fz  (N)  normal/support force (upward +)
 *   3 Tx  (N·m) rolling torque about x
 *   4 Ty  (N·m) rolling torque about y
 */

#define S_FUNCTION_NAME  ballplateforces
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>

/* Parameter/Signal access helpers */
#define PAR(i)   (*mxGetPr(ssGetSFcnParam(S,(i))))
#define U(i)     (*uPtrs[(i)])

/* ----------------- Dialog parameters (7 total) ------------------------- */
#define rpla    PAR(0) /* Plate radius [m] */
#define zpla    PAR(1) /* Plate thickness [m] */
#define rbal    PAR(2) /* Ball radius [m] */
#define Kpen    PAR(3) /* Penetration spring [N/m] */
#define Dpen    PAR(4) /* Penetration damping [N·s/m] */
#define mustat  PAR(5) /* Static friction coefficient [-] */
#define vthr    PAR(6) /* Slip threshold speed [m/s] */

/* ----------------- Packed input vector -------------------------------- */
#define xpos    U(0)
#define vx      U(1)
#define ypos    U(2)
#define vy      U(3)
#define pz      U(4)
#define vz      U(5)
#define wx      U(6)
#define wy      U(7)
#define Tsim    U(8)

/* ====================================================================== */
/* Initialization                                                         */
/* ====================================================================== */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 7);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 9);
    ssSetInputPortDirectFeedThrough(S, 0, 1);  /* boolean: yes, uses inputs in mdlOutputs */

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 5);

    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
    /* no states */
}
#endif

/* ====================================================================== */
/* Outputs                                                                */
/* ====================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T              *y    = ssGetOutputPortRealSignal(S,0);
    InputRealPtrsType   uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    real_T cgap, Fz;
    real_T vxslip, vyslip;
    real_T mux, muy, Ffx, Ffy;
    real_T Tx, Ty;

    /* Contact gap at just-touching configuration (top surface) */
    cgap = zpla*0.5 + rbal;

    /* Use effective radius so the ball loses support when its edge reaches the rim.
       If rbal >= rpla, fall back to rpla so the contact won't vanish entirely. */
    {
        /* const real_T r_eff = (rpla > rbal) ? (rpla - rbal) : rpla;*/
        const real_T r_eff = rpla;
        const real_T r2    = xpos*xpos + ypos*ypos;

        if ( (pz < cgap) &&
             (r2 <= r_eff*r_eff) &&
             (Tsim > 1.0e-3) )
        {
            /* Linear spring-damper penalty for normal force */
            Fz = -Kpen*(pz - cgap) - Dpen*vz;
            if (Fz < 0.0) Fz = 0.0;  /* no adhesion */
        }
        else
        {
            Fz = 0.0;
        }
    }

    /* Slip velocities at contact (signs consistent with original code) */
    vxslip = -vx + rbal*wy;
    vyslip = -vy - rbal*wx;

    /* Friction coefficients with linear deadzone around zero slip */
    if (fabs(vxslip) <= vthr)      mux = mustat * (vxslip / vthr);
    else if (vxslip >  vthr)       mux = mustat;
    else                           mux = -mustat;

    if (fabs(vyslip) <= vthr)      muy = mustat * (vyslip / vthr);
    else if (vyslip >  vthr)       muy = mustat;
    else                           muy = -mustat;

    /* Friction forces proportional to normal load */
    Ffx = mux * Fz;
    Ffy = muy * Fz;

    /* Rolling torques from tangential forces */
    Tx =  rbal * Ffy;   /* about x */
    Ty = -rbal * Ffx;   /* about y */

    /* Outputs */
    y[0] = Ffx;
    y[1] = Ffy;
    y[2] = Fz;
    y[3] = Tx;
    y[4] = Ty;
}

#define MDL_DERIVATIVES
#if defined(MDL_DERIVATIVES)
static void mdlDerivatives(SimStruct *S)
{
    /* no continuous states */
}
#endif

static void mdlTerminate(SimStruct *S)
{
    /* nothing to free */
}

/* ----------------------- Required trailer ------------------------------ */
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
