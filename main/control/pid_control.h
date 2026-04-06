#include "pid.h"
#include "pwm_control.h"
#include "comms/msg_send.h"

static int64_t last_time = 0;

msg_control_mode_t control_mode = CFG_MODE_RATE_HOLD;

// Outer loop (angle)
static tPID roll_angle_pid = PID_DEFAULTS;
static tPID pitch_angle_pid = PID_DEFAULTS;
static tPID yaw_angle_pid = PID_DEFAULTS;

// Inner loop (rate)
static tPID roll_rate_pid = PID_DEFAULTS;
static tPID pitch_rate_pid = PID_DEFAULTS;
static tPID yaw_rate_pid = PID_DEFAULTS;

// Goal setpoint set by external commands
// Can be either degrees or degrees per second, depending on control_mode
float roll_target  = 0.0f;
float pitch_target = 0.0f;
float yaw_target   = 0.0f;

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
    // Prevent windup while unpowered
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
    float e_roll  = roll_target  - roll_deg;
    float e_pitch = pitch_target - pitch_deg;
    float e_yaw   = yaw_target   - yaw_deg;

    float roll_rate_target = 0;
    float pitch_rate_target = 0;
    float yaw_rate_target = 0;

    roll_angle_pid.fIn  = e_roll;
    roll_angle_pid.m_calc(&roll_angle_pid);

    pitch_angle_pid.fIn = e_pitch;
    pitch_angle_pid.m_calc(&pitch_angle_pid);

    yaw_angle_pid.fIn   = e_yaw;
    yaw_angle_pid.m_calc(&yaw_angle_pid);

    if (control_mode == CFG_MODE_ANGLE_HOLD)
    {
        // Outer + inner loop
        roll_rate_target = roll_angle_pid.fOut;
        pitch_rate_target = pitch_angle_pid.fOut;
        yaw_rate_target = yaw_angle_pid.fOut;
    }
    else if (control_mode == CFG_MODE_RATE_HOLD)
    {
        // Inner loop only
        roll_rate_target = roll_target;
        pitch_rate_target = pitch_target;
        yaw_rate_target = yaw_target;
    }

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

    // printf("Orientation target: %f %f %f\n", roll_target, pitch_target, yaw_target);
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
    if (control_mode == CFG_MODE_RATE_HOLD || control_mode == CFG_MODE_ANGLE_HOLD)
    {
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
    
    
}

// Update the drone's orientation setpoints
void pid_set_targets(float roll, float pitch, float yaw, float throttle)
{
    roll_target  = roll;
    pitch_target = pitch;
    yaw_target   = yaw;
    throttle     = throttle;
}

void populate_pid_state_msg(pid_state_t* pid_state, tPID* pid)
{
    pid_state->dt_sec = pid->fDtSec;
    pid_state->prev_input = pid->fPrevIn;
    pid_state->integral = pid->fIntegral;
    pid_state->input = pid->fIn;
    pid_state->output = pid->fOut;
}

void handle_pid_telemetry(const void *payload)
{
    msg_header_t msg_header = {
        .msg_type = MSG_PID_INFO,
        .payload_len = sizeof(msg_pid_info_t),
    };

    msg_pid_info_t msg_pid_info = {0};
    populate_pid_state_msg(&msg_pid_info.rate_roll_pid_state, &roll_rate_pid);
    populate_pid_state_msg(&msg_pid_info.rate_pitch_pid_state, &pitch_rate_pid);
    populate_pid_state_msg(&msg_pid_info.rate_yaw_pid_state, &yaw_rate_pid);

    populate_pid_state_msg(&msg_pid_info.angle_roll_pid_state, &roll_angle_pid);
    populate_pid_state_msg(&msg_pid_info.angle_pitch_pid_state, &pitch_angle_pid);
    populate_pid_state_msg(&msg_pid_info.angle_yaw_pid_state, &yaw_angle_pid);
    
    send_message(msg_header, &msg_pid_info);
}

void handle_control_mode_msg(const void *payload)
{
    msg_control_mode_t msg = *(msg_control_mode_t*)payload;

    control_mode = msg;
}

void handle_control_target_msg(const void *payload)
{
    msg_control_target_t* msg = (msg_control_target_t*)payload;

    throttle_cmd = msg->throttle;
    roll_target = msg->roll;
    pitch_target = msg->pitch;
    yaw_target = msg->yaw;
}

void populate_pid_from_cfg_msg(tPID* pid, pid_cfg_t* pid_cfg)
{
    pid->fKp = pid_cfg->kP;
    pid->fKi = pid_cfg->kI;
    pid->fKd = pid_cfg->kD;

    pid->fUpIntLim = pid_cfg->int_up_lim;
    pid->fLowIntLim = pid_cfg->int_low_lim;
}

void handle_cfg_pid(const void *payload)
{
    msg_cfg_pid_t* msg = (msg_cfg_pid_t*)payload;

    populate_pid_from_cfg_msg(&roll_rate_pid, &msg->rate_roll_pid);
    populate_pid_from_cfg_msg(&pitch_rate_pid, &msg->rate_pitch_pid);
    populate_pid_from_cfg_msg(&yaw_rate_pid, &msg->rate_yaw_pid);

    populate_pid_from_cfg_msg(&roll_angle_pid, &msg->angle_roll_pid);
    populate_pid_from_cfg_msg(&pitch_angle_pid, &msg->angle_pitch_pid);
    populate_pid_from_cfg_msg(&yaw_angle_pid, &msg->angle_yaw_pid);

    pid_reset(&roll_angle_pid);
    pid_reset(&pitch_angle_pid);
    pid_reset(&yaw_angle_pid);

    pid_reset(&roll_rate_pid);
    pid_reset(&pitch_rate_pid);
    pid_reset(&yaw_rate_pid);
}
