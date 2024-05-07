
#ifndef ANTICOPTER_CAMERA
#define ANTICOPTER_CAMERA

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>
#include <esp_camera.h>

#define CONFIG_XCLK_FREQ 10000000

// LED pins and params
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 4

// Camera pins
#define CAM_PIN_PWDN -1  // power down is not used
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_XCLK 13
#define CAM_PIN_SIOD 45
#define CAM_PIN_SIOC 48

#define CAM_PIN_D9 14
#define CAM_PIN_D8 12
#define CAM_PIN_D7 11
#define CAM_PIN_D6 9
#define CAM_PIN_D5 3
#define CAM_PIN_D4 18
#define CAM_PIN_D3 8
#define CAM_PIN_D2 46
#define CAM_PIN_VSYNC 47
#define CAM_PIN_HREF 21
#define CAM_PIN_PCLK 10

static esp_err_t init_camera(void)
{
    camera_config_t camera_config = {.pin_pwdn = CAM_PIN_PWDN,
                                     .pin_reset = CAM_PIN_RESET,
                                     .pin_xclk = CAM_PIN_XCLK,
                                     .pin_sccb_sda = CAM_PIN_SIOD,
                                     .pin_sccb_scl = CAM_PIN_SIOC,

                                     .pin_d7 = CAM_PIN_D9,
                                     .pin_d6 = CAM_PIN_D8,
                                     .pin_d5 = CAM_PIN_D7,
                                     .pin_d4 = CAM_PIN_D6,
                                     .pin_d3 = CAM_PIN_D5,
                                     .pin_d2 = CAM_PIN_D4,
                                     .pin_d1 = CAM_PIN_D3,
                                     .pin_d0 = CAM_PIN_D2,
                                     .pin_vsync = CAM_PIN_VSYNC,
                                     .pin_href = CAM_PIN_HREF,
                                     .pin_pclk = CAM_PIN_PCLK,

                                     .xclk_freq_hz = CONFIG_XCLK_FREQ,
                                     .ledc_timer = LEDC_TIMER_0,
                                     .ledc_channel = LEDC_CHANNEL_0,

                                     .pixel_format = PIXFORMAT_JPEG,
                                     .frame_size = FRAMESIZE_VGA,

                                     .jpeg_quality = 10,
                                     .fb_count = 2,
                                     .grab_mode = CAMERA_GRAB_LATEST}; // CAMERA_GRAB_LATEST. Sets when buffers should be filled
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

#endif