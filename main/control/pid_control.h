#include "fp_pid.h"
#include "pwm_control.h"

//
// ===================== PID CONTROLLERS =====================
//

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
float throttle_cmd = 50.0f;

extern float orientation[3];        // roll, pitch, yaw (deg)
extern float angular_rate_mdps[3];  // deg/sec

//
// ===================== PID INITIALIZATION =====================
//
void pid_init()
{
    last_time = 0.1f;
    //
    // --- Angle PIDs (outer loop) ---
    //
    roll_angle_pid.fKp = 0.1f;
    roll_angle_pid.fKi = 0.0f;
    roll_angle_pid.fKd = 0.0f;
    roll_angle_pid.fUpOutLim  =  100.0f;
    roll_angle_pid.fLowOutLim = -100.0f;

    pitch_angle_pid = roll_angle_pid;
    yaw_angle_pid = roll_angle_pid;

    //
    // --- Rate PIDs (inner loop) ---
    //
    roll_rate_pid.fKp = 0.1f;
    roll_rate_pid.fKi = 0.01f;
    roll_rate_pid.fKd = 0.0f;
    roll_rate_pid.fUpOutLim  =  100.0f;
    roll_rate_pid.fLowOutLim = -100.0f;

    pitch_rate_pid = roll_rate_pid;
    yaw_rate_pid   = roll_rate_pid;      // yaw D term can be lower
}

//
// ===================== MAIN PID TICK (CALL EVERY LOOP) =====================
//
void pid_tick()
{
    int64_t now = esp_timer_get_time();
    double dt = (double)(now - last_time) / 1e6;

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

    float roll_rate  = angular_rate_mdps[0] / 1000;
    float pitch_rate = angular_rate_mdps[1] / 1000;
    float yaw_rate   = angular_rate_mdps[2] / 1000;

    //
    // 2. Angle error -> desired rates (outer loop)
    //
    float e_roll  = roll_target_deg  - roll_deg;
    float e_pitch = pitch_target_deg - pitch_deg;
    float e_yaw   = yaw_target_deg   - yaw_deg;

    roll_angle_pid.fIn  = e_roll;
    roll_angle_pid.m_calc(&roll_angle_pid);
    float roll_rate_target = roll_angle_pid.fOut;

    pitch_angle_pid.fIn = e_pitch;
    pitch_angle_pid.m_calc(&pitch_angle_pid);
    float pitch_rate_target = pitch_angle_pid.fOut;

    yaw_angle_pid.fIn   = e_yaw;
    yaw_angle_pid.m_calc(&yaw_angle_pid);
    float yaw_rate_target = yaw_angle_pid.fOut;

    //
    // 3. Rate error -> control torque (inner loop)
    //
    float roll_err_rate  = roll_rate_target  - roll_rate;
    float pitch_err_rate = pitch_rate_target - pitch_rate;
    float yaw_err_rate   = yaw_rate_target   - yaw_rate;

    roll_rate_pid.fIn  = roll_err_rate;
    roll_rate_pid.m_calc(&roll_rate_pid);
    float roll_cmd = roll_rate_pid.fOut;

    pitch_rate_pid.fIn = pitch_err_rate;
    pitch_rate_pid.m_calc(&pitch_rate_pid);
    float pitch_cmd = pitch_rate_pid.fOut;

    yaw_rate_pid.fIn   = yaw_err_rate;
    yaw_rate_pid.m_calc(&yaw_rate_pid);
    float yaw_cmd = yaw_rate_pid.fOut;

    printf("Orientation target: %f %f %f\n", roll_target_deg, pitch_target_deg, yaw_target_deg);
    printf("Orientation: %f %f %f\n", roll_deg, pitch_deg, yaw_deg);
    printf("Angular rate target: %f %f %f\n", roll_rate_target, pitch_rate_target, yaw_rate_target);
    printf("Angular rate: %f %f %f\n", roll_cmd, pitch_cmd, yaw_cmd);

    //
    // 4. Mixer (quad X)
    //
    // roll = rotate around x
    // pitch = rotate around y
    // yaw = rotate around z
    float m3 = throttle_cmd - roll_cmd + pitch_cmd - yaw_cmd;
    float m2 = throttle_cmd - roll_cmd - pitch_cmd + yaw_cmd;
    float m1 = throttle_cmd + roll_cmd - pitch_cmd - yaw_cmd;
    float m0 = throttle_cmd + roll_cmd + pitch_cmd + yaw_cmd;


    // Clamp 0-100%
    if (m0 < 0) m0 = 0; 
    if (m0 > 100) m0 = 100;
    if (m1 < 0) m1 = 0; 
    if (m1 > 100) m1 = 100;
    if (m2 < 0) m2 = 0; 
    if (m2 > 100) m2 = 100;
    if (m3 < 0) m3 = 0; 
    if (m3 > 100) m3 = 100;


    //
    // 5. Output to motors
    //
    set_motor_pwm(0, (int)m0);
    set_motor_pwm(1, (int)m1);
    set_motor_pwm(2, (int)m2);
    set_motor_pwm(3, (int)m3);
}

//
// =============== Set external control inputs =================
//
void pid_set_targets(float roll, float pitch, float yaw, float throttle)
{
    roll_target_deg  = roll;
    pitch_target_deg = pitch;
    yaw_target_deg   = yaw;
    throttle_cmd     = throttle;
}
