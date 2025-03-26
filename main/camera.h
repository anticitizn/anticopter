
#ifndef ANTICOPTER_CAMERA
#define ANTICOPTER_CAMERA

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>
#include <esp_camera.h>
#include <esp_timer.h>
#include <esp_log.h>

#define CONFIG_XCLK_FREQ 20000000

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

size_t _jpg_buf_len;
uint8_t *_jpg_buf;

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
                                     .frame_size = FRAMESIZE_HD,

                                     .jpeg_quality = 10,
                                     .fb_count = 2,
                                     .grab_mode = CAMERA_GRAB_LATEST}; // CAMERA_GRAB_LATEST. Sets when buffers should be filled
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_vflip(sensor, true);
    sensor->set_hmirror(sensor, true);

    return ESP_OK;
}

void cam_take_picture()
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    
    static int64_t last_frame = 0;

    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    fb = esp_camera_fb_get();
    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        res = ESP_FAIL;
        return;
    }

    // Camera format is in JPEG already
    _jpg_buf_len = fb->len;
    _jpg_buf = fb->buf;

    esp_camera_fb_return(fb);

    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)", (unsigned int)(_jpg_buf_len / 1024), (unsigned int)frame_time,
                1000.0 / (unsigned int)frame_time);


    last_frame = 0;    
}

#endif