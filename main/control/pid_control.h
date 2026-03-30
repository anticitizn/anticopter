#include "pid.h"
#include "pwm_control.h"

static int64_t last_time = 0;

// Outer loop (angle)
static tPID roll_angle_pid = PID_DEFAULTS;
static tPID pitch_angle_pid = PID_DEFAULTS;
static tPID yaw_angle_pid = PID_DEFAULTS;

// Inner loop (rate)
static tPID roll_rate_pid = PID_DEFAULTS;
static tPID pitch_rate_pid = PID_DEFAULTS;
static tPID yaw_rate_pid = PID_DEFAULTS;

// Setpoints (set by external commands)
float roll_target_deg  = 0.0f;
float pitch_target_deg = 0.0f;
float yaw_target_deg   = 0.0f;

// Throttle input (0-100%)
float throttle_cmd = 0.0f;

extern float orientation[3];       // orientation (roll, pitch, yaw in deg)
extern float angular_rate_dps[3];  // angular rate (roll, pitch, yaw in deg/s)

//
// ===================== PID INITIALIZATION =====================
//
void pid_init()
{
    last_time = esp_timer_get_time();

    //
    // -------- ANGLE (OUTER) LOOP --------
    //
    roll_angle_pid.fKp = 1.0f;
    roll_angle_pid.fKi = 0.00f;
    roll_angle_pid.fKd = 0.05f;
    roll_angle_pid.fUpOutLim  =  100.0f;
    roll_angle_pid.fLowOutLim = -100.0f;
    roll_angle_pid.fUpIntLim  =  0.0f;
    roll_angle_pid.fLowIntLim = 0.0f;

    pitch_angle_pid = roll_angle_pid;

    yaw_angle_pid = roll_angle_pid;
    yaw_angle_pid.fKp = 0.8f;
    yaw_angle_pid.fKi = 0.0f;
    yaw_angle_pid.fKd = 0;
    yaw_angle_pid.fUpIntLim  =  0.0f;
    yaw_angle_pid.fLowIntLim = 0.0f;

    //
    // -------- RATE (INNER) LOOP --------
    //
    roll_rate_pid.fKp = 0.1f;
    roll_rate_pid.fKi = 0;
    roll_rate_pid.fKd = 0;
    roll_rate_pid.fUpOutLim  =  100.0f;
    roll_rate_pid.fLowOutLim = -100.0f;
    roll_rate_pid.fUpIntLim  =  0.0f;
    roll_rate_pid.fLowIntLim =  0.0f;

    pitch_rate_pid = roll_rate_pid;

    yaw_rate_pid = roll_rate_pid;
    yaw_rate_pid.fKp = 0.20f;
    yaw_rate_pid.fKi = 0.15f;
    yaw_rate_pid.fKd = 0.000f;

    yaw_rate_pid.fUpIntLim  =  15.0f;
    yaw_rate_pid.fLowIntLim =  -15.0f;
}

//
// ===================== MAIN PID TICK =====================
//
void pid_tick()
{
    if (throttle_cmd < 1)
    {
        pid_reset(&roll_angle_pid);
        pid_reset(&pitch_angle_pid);
        pid_reset(&yaw_angle_pid);

        pid_reset(&roll_rate_pid);
        pid_reset(&pitch_rate_pid);
        pid_reset(&yaw_rate_pid);
    }

    int64_t now = esp_timer_get_time();
    double dt = (double)(now - last_time) / 1e6;
    // printf("Dt: %f\n", dt);

    last_time = now;

    roll_angle_pid.fDtSec = dt;
    pitch_angle_pid.fDtSec = dt;
    yaw_angle_pid.fDtSec = dt;

    roll_rate_pid.fDtSec = dt;
    pitch_rate_pid.fDtSec = dt;
    yaw_rate_pid.fDtSec = dt;

    //
    // 1. Read current state
    //
    float roll_deg  = orientation[0];
    float pitch_deg = orientation[1];
    float yaw_deg   = orientation[2];

    float roll_rate  = angular_rate_dps[0];
    float pitch_rate = angular_rate_dps[1];
    float yaw_rate   = angular_rate_dps[2];

    //
    // 2. Angle error -> desired rates (outer loop)
    //
    float e_roll  = roll_target_deg  - roll_deg;
    float e_pitch = pitch_target_deg - pitch_deg;
    float e_yaw   = yaw_target_deg   - yaw_deg;

    roll_angle_pid.fIn  = e_roll;
    roll_angle_pid.m_calc(&roll_angle_pid);
    // Disable outer loop for testing
    //float roll_rate_target = roll_angle_pid.fOut;
    float roll_rate_target = roll_target_deg;

    pitch_angle_pid.fIn = e_pitch;
    pitch_angle_pid.m_calc(&pitch_angle_pid);
    // Disable outer loop for testing
    //float pitch_rate_target = pitch_angle_pid.fOut;
    float pitch_rate_target = pitch_target_deg;

    yaw_angle_pid.fIn   = e_yaw;
    yaw_angle_pid.m_calc(&yaw_angle_pid);
    // Disable outer loop for testing
    //float yaw_rate_target = yaw_angle_pid.fOut;
    float yaw_rate_target = yaw_target_deg;

    //
    // 3. Rate error -> control torque (inner loop)
    //
    float roll_err_rate  = roll_rate_target  - roll_rate;
    float pitch_err_rate = pitch_rate_target - pitch_rate;
    float yaw_err_rate   = yaw_rate_target   - yaw_rate;
    // printf("error rate: roll %f, pitch %f, yaw %f\n", roll_err_rate, pitch_err_rate, yaw_err_rate);

    roll_rate_pid.fIn  = roll_err_rate;
    roll_rate_pid.m_calc(&roll_rate_pid);
    float roll_cmd = roll_rate_pid.fOut;

    pitch_rate_pid.fIn = pitch_err_rate;
    pitch_rate_pid.m_calc(&pitch_rate_pid);
    float pitch_cmd = pitch_rate_pid.fOut;

    yaw_rate_pid.fIn   = yaw_err_rate;
    yaw_rate_pid.m_calc(&yaw_rate_pid);
    float yaw_cmd = yaw_rate_pid.fOut;
    printf("rate command: roll %f, pitch %f, yaw %f\n", roll_cmd, pitch_cmd, yaw_cmd);

    // printf("Orientation target: %f %f %f\n", roll_target_deg, pitch_target_deg, yaw_target_deg);
    // printf("Orientation: %f %f %f\n", roll_deg, pitch_deg, yaw_deg);
    // printf("Angular rate target: %f %f %f\n", roll_rate_target, pitch_rate_target, yaw_rate_target);
    // printf("Angular rate command: %f %f %f\n", roll_cmd, pitch_cmd, yaw_cmd);
    // printf("Actual angular rate: %f %f %f\n", roll_rate, pitch_rate, yaw_rate);

    //
    // 4. Mixer
    //
    // roll = rotate around x
    // pitch = rotate around y
    // yaw = rotate around z

    float m0 = throttle_cmd - roll_cmd + pitch_cmd - yaw_cmd;
    float m1 = throttle_cmd - roll_cmd - pitch_cmd + yaw_cmd;
    float m2 = throttle_cmd + roll_cmd - pitch_cmd - yaw_cmd;
    float m3 = throttle_cmd + roll_cmd + pitch_cmd + yaw_cmd;


    // Clamp 0-100%
    if (m0 < 0) m0 = 0; 
    if (m0 > 100) m0 = 100;
    if (m1 < 0) m1 = 0; 
    if (m1 > 100) m1 = 100;
    if (m2 < 0) m2 = 0; 
    if (m2 > 100) m2 = 100;
    if (m3 < 0) m3 = 0; 
    if (m3 > 100) m3 = 100;

    printf("Motors PWM: %f %f %f %f\n", m0, m1, m2, m3);

    //
    // 5. Output to motors
    //
    if (throttle_cmd > 1)
    {
        set_motor_pwm(0, m0);
        set_motor_pwm(1, m1);
        set_motor_pwm(2, m2);
        set_motor_pwm(3, m3);
    }
    else
    {
        set_motor_pwm(0, 0);
        set_motor_pwm(1, 0);
        set_motor_pwm(2, 0);
        set_motor_pwm(3, 0);
    }
    
}

// Update the drone's orientation setpoints
void pid_set_targets(float roll, float pitch, float yaw, float throttle)
{
    roll_target_deg  = roll;
    pitch_target_deg = pitch;
    yaw_target_deg   = yaw;
    throttle_cmd     = throttle;
}
