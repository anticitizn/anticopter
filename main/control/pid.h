#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct PID tPID;

struct PID 
{
    // Gains
    float fKp; // Proportional term
    float fKi; // Integral term
    float fKd; // Derivative term

    // Timing
    float fDtSec; //Sample time (s), set before each pid_calc() call

    // I/O
    float fIn;         // Input: error (setpoint - measurement)
    float fOut;        // Output: control signal

    // Internal state
    float fPrevIn;     // Previous input (used for D term)
    float fIntegral;   // Integral accumulator

    // Output limits / clamping values
    float fUpOutLim;
    float fLowOutLim;

    // Integral limits / integral clamping values
    float fUpIntLim;
    float fLowIntLim;

    // Function pointer used to calculate this PID controller's internals
    // maybe remove this later, but useful for testing now
    void (*m_calc)(tPID *pid);
};

// Core PID functions
void pid_calc(tPID *pid);
void pid_reset(tPID *pid);
void pid_set_gains(tPID *pid, float kp, float ki, float kd);

// Default PID initializer with all values set to 0 and method pointer populated
#define PID_DEFAULTS                                         \
{                                                            \
    .fKp        = 0.0f,                                      \
    .fKi        = 0.0f,                                      \
    .fKd        = 0.0f,                                      \
    .fDtSec     = 0.0f,                                      \
    .fIn        = 0.0f,                                      \
    .fOut       = 0.0f,                                      \
    .fPrevIn    = 0.0f,                                      \
    .fIntegral  = 0.0f,                                      \
    .fUpOutLim  = 0.0f,                                      \
    .fLowOutLim = 0.0f,                                      \
    .fUpIntLim  = 0.0f,                                      \
    .fLowIntLim = 0.0f,                                      \
    .m_calc     = pid_calc                                   \
}

#endif /* PID_H */
