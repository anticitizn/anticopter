#include "pid.h"

void pid_reset(tPID *pid)
{
    if (!pid) 
    {
        return;
    }

    pid->fIn       = 0.0f;
    pid->fOut      = 0.0f;
    pid->fPrevIn   = 0.0f;
    pid->fIntegral = 0.0f;
}

void pid_set_gains(tPID *pid, float kp, float ki, float kd)
{
    if (!pid) 
    {
        return;
    }

    pid->fKp = kp;
    pid->fKi = ki;
    pid->fKd = kd;
}

// Basic PID calculation with integral and output clamping
void pid_calc(tPID *pid)
{
    if (!pid) 
    {
        return;
    }

    float err = pid->fIn;
    float dt  = pid->fDtSec;

    if (dt < 1e-6f) 
    {
        dt = 0.0f;
    }

    // Proportional
    float p_term = pid->fKp * err;

    // Derivative (on measurement)
    float d_term = 0.0f;
    if (dt > 0.0f && pid->fKd != 0.0f) 
    {
        float dmeas = (pid->fPrevIn - err) / dt;  
        d_term = pid->fKd * dmeas;
    }
    pid->fPrevIn = err;


    // Integral pre-computation
    float i_term = pid->fIntegral;
    if (dt > 0.0f && pid->fKi != 0.0f) 
    {
        i_term += pid->fKi * err * dt;
    }

    // Unclamped output
    float out_raw = p_term + i_term + d_term;

    // Saturation checks
    float out_sat = out_raw;
    if (out_sat > pid->fUpOutLim)  out_sat = pid->fUpOutLim;
    if (out_sat < pid->fLowOutLim) out_sat = pid->fLowOutLim;

    // Integral back-calculation (anti-windup)
    if (pid->fKi != 0.0f) 
    {
        float aw_err = out_sat - out_raw;

        // Back-calculation gain
        float kaw = 0.5f; 

        pid->fIntegral = i_term + kaw * aw_err;

        // Apply integral part clamping
        if (pid->fUpIntLim > pid->fLowIntLim) 
        {
            if (pid->fIntegral > pid->fUpIntLim)  pid->fIntegral = pid->fUpIntLim;
            if (pid->fIntegral < pid->fLowIntLim) pid->fIntegral = pid->fLowIntLim;
        }
    } 
    else 
    {
        pid->fIntegral = i_term;
    }

    pid->fOut = out_sat;
}

