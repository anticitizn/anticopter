#ifndef ANTICOPTER_MSG
#define ANTICOPTER_MSG

typedef enum 
{
    // Protocol messages
    MSG_INIT               = 0,
    MSG_ACK                = 1,
    MSG_ERROR              = 2,

    // Control
    MSG_ARM                = 10,
    MSG_DISARM             = 11,
    MSG_CONTROL_MODE       = 12,
    MSG_CONTROL_TARGET     = 13,
    MSG_MOTOR_PWM          = 14,

    // Configuration
    MSG_CFG_LED            = 30,
    MSG_CFG_CAMERA         = 31,
    MSG_CFG_PID            = 32,
    MSG_CFG_IMU            = 33,

    // Telemetry
    MSG_IMU                = 40,
    MSG_LOG_TEXT           = 41,
    MSG_PID_INFO           = 42,

    // Camera
    MSG_GET_IMAGE          = 50,
    MSG_START_RECORDING    = 51,
    MSG_STOP_RECORDING     = 52,
} msg_type_t;

// Communications init data
typedef struct
{
    uint32_t timestamp;
} msg_init_t;

// Control mode
typedef enum
{
    CFG_MODE_PWM_HOLD,      // Motor PWMs set directly by MSG_MOTOR_PWM
    CFG_MODE_RATE_HOLD,     // Only inner (angular rate PID) controller loop is active, control target defines goal angular rate
    CFG_MODE_ANGLE_HOLD,    // Both inner (angular rate) and outer (angle orientation) contollers are active, control target defines goal orientation
} msg_control_mode_t;

// Control goal target
typedef struct
{
    float throttle;
    float roll;
    float pitch;
    float yaw;
} msg_control_target_t;

// Individual motor PWM control
typedef struct
{
    uint8_t pwm[4];
} msg_control_motor_pwm_t;

// IMU configuration
typedef struct
{
    float Kp_acc;
    float Kp_mag;
    float Ki_acc;
    float alpha_mag;

    float mag_bias[3];
    float mag_scale[3];
} msg_cfg_imu_t;

// PID configuration
typedef struct
{
    float kP; // Proportional term
    float kI; // Integral term
    float kD; // Derivative term

    // Output limits / clamping values
    float out_up_lim;
    float out_low_lim;

    // Integral limits / integral clamping values
    float int_up_lim;
    float low_int_lim;
} pid_cfg_t;

typedef struct
{
    pid_cfg_t rate_roll_pid;
    pid_cfg_t rate_pitch_pid;
    pid_cfg_t rate_yaw_pid;

    pid_cfg_t angle_roll_pid;
    pid_cfg_t angle_pitch_pid;
    pid_cfg_t angle_yaw_pid;
} msg_cfg_pid_t;

// LED configuration
typedef struct
{
    uint32_t pwm[4][3];
} msg_cfg_led_t;

// Camera configuration
typedef enum
{
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_HVGA,     // 480x320
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_HD,       // 1280x720
    FRAMESIZE_UXGA,     // 1600x1200
} camera_resolution_t;

typedef struct
{
    uint8_t compression_rate; // 0-63, lower value = higher image quality
    uint8_t camera_resolution; // See camera_resolution_t
} msg_cfg_camera_t;

// IMU telemetry
typedef struct 
{   
    // Accelerometer/gyroscope data
    float acceleration_mg[3];
    float angular_rate_dps[3];
    float orientation[3];

    // Magnetometer data
    float magnetic_mG[3];
    float mag_norm[3];

    float temperature_degC;
} msg_imu_t;

// PID telemetry
typedef struct
{
    float dt_sec;
    float prev_input;
    float integral;
    float input;
    float output;
} pid_state_t;

typedef struct
{
    pid_state_t rate_roll_pid_state;
    pid_state_t rate_pitch_pid_state;
    pid_state_t rate_yaw_pid_state;

    pid_state_t angle_roll_pid_state;
    pid_state_t angle_pitch_pid_state;
    pid_state_t angle_yaw_pid_state;
} msg_pid_info_t;

// Log telemetry
typedef struct
{
    char* text;
} msg_log_t;

// Camera telemetry
typedef struct
{
    uint8_t* image_data;
} msg_camera_t;

#endif
