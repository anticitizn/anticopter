#ifndef ANTICOPTER_COMMS_HEADER
#define ANTICOPTER_COMMS_HEADER

// Comms message header, 20 bytes long
typedef struct 
{
    uint16_t magic = 0x4452; // Magic number
    uint8_t  version;        // Protocol version number
    uint16_t msg_type;       // Message type; see msg_type_t  
    uint8_t  flags;          // Message flags, see msg_flags_t
    uint32_t seq;            // Message sequence number (incremented separately for each sender)
    uint32_t frag_seq;       // If message is fragmented, messages that get split retain the same sequence number, 
                             // and the fragmentation sequence number is incremented instead
    uint32_t timestamp;      // Microseconds since communication was established
    uint16_t payload_len;    // Length of the packet payload in bytes
} __attribute__((packed)) comms_header;

typedef enum
{
    MSG_FLAG_FRAGMENTED = 1 << 0, // If set, the contained data is too large to fit within one packet and has been split
                                  // across multiple packets, which means that frag_seq has to be considered.
                                  // This can typically only happen for image and log data
    MSG_FLAG_WRITE      = 1 << 1, // If set, the sender intends to write data to the receiver
                                  // If zero, the receiver can safely ignore any attached payload, 
                                  // but is typically expected to return data as indicated by msg_type
} msg_flags_t;

typedef enum 
{
    // Protocol messages
    MSG_INIT               = 0,
    MSG_ACK                = 1,
    MSG_ERROR              = 2,

    // Control
    MSG_ARM                = 10,
    MSG_DISARM             = 11,
    MSG_SET_CONTROL_MODE   = 12,
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

// Control mode
typedef struct
{
    uint32_t timestamp;
} msg_init_t;

// Control mode
typedef struct
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
    camera_resolution_t camera_resolution;
} msg_cfg_camera_t;

// IMU telemetry
typedef struct 
{   
    // Accelerometer/gyroscope data
    float acceleration_mg[3];
    float angular_rate_dps[3];
    float orientation[3];

    // Magnetometer data
    float magnetic_mG[3] = {0};
    float mag_norm[3] = {0};

    float temperature_degC;
} msg_imu_t;

// PID telemetry
typedef struct
{
    float dt_sec;
    float prev_integral;
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
} msg_pid_t;

// Log telemetry
typedef struct
{
    char* text;
} msg_log_t;

// Camera telemetry
typedef struct
{
    uint8_t* image_data;
} msg_log_t;

#endif
