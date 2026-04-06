
#ifndef ANTICOPTER_MSG_HANDLER_H
#define ANTICOPTER_MSG_HANDLER_H

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>
#include <sys/param.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "msg_header.h"
#include "msg_type.h"

#include "../imu.h"
#include "../control/pwm_control.h"
#include "../control/pid_control.h"
#include "../led.h"
#include "../camera.h"

uint32_t us_since_comms_init = 0;

extern comms_config_t comms_config;

typedef void (*msg_handler_t)(const void *payload);

static const msg_handler_t msg_read_handlers[] = 
{
    [MSG_IMU]             = handle_imu_telemetry,
    //[MSG_LOG_TEXT]        = handle_log_telemetry,
    [MSG_PID_INFO]        = handle_pid_telemetry,
    [MSG_CAMERA]          = handle_camera_telemetry,
};

static const msg_handler_t msg_write_handlers[] = 
{
    // [MSG_INIT]            = handle_comms_init_msg,
    [MSG_ARM]             = handle_arm_msg,
    [MSG_DISARM]          = handle_disarm_msg,
    [MSG_CONTROL_MODE]    = handle_control_mode_msg,
    [MSG_CONTROL_TARGET]  = handle_control_target_msg,
    [MSG_MOTOR_PWM]       = handle_motor_pwm_msg,
    [MSG_CFG_LED]         = handle_cfg_led,
    [MSG_CFG_CAMERA]      = handle_cfg_camera,
    [MSG_CFG_PID]         = handle_cfg_pid,
    [MSG_CFG_IMU]         = handle_cfg_imu,
    [MSG_START_RECORDING] = handle_camera_start_recording,
    [MSG_STOP_RECORDING]  = handle_camera_stop_recording,
};

void handle_packet(msg_header_t *header, uint8_t *payload)
{
    if (header->magic_number != comms_config.magic_number)
    {
        return;
    }
    
    const msg_handler_t *table = (header->flags & MSG_FLAG_WRITE) ? msg_write_handlers : msg_read_handlers;

    if (header->msg_type >= ARRAY_SIZE(msg_write_handlers))
    {
        // Out of range, which means that the received msg_type has a larger value than any currently known; possible version mismatch?
        return; 
    }

    msg_handler_t handler = table[header->msg_type];
    if (handler)
    {
        handler(payload);
    }
}

// TO-DO: Finalize implementation
void handle_comms_init_msg(const void *payload)
{
    msg_init_t* msg = (msg_init_t*)payload;

    us_since_comms_init = msg->timestamp;
}

#endif
