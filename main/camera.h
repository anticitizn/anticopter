
#ifndef ANTICOPTER_CAMERA
#define ANTICOPTER_CAMERA

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>
#include <esp_camera.h>
#include <esp_timer.h>
#include <esp_log.h>

#include "comms/msg_send.h"

#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"

#define CONFIG_XCLK_FREQ 20000000

// Camera pins
#define CAM_PIN_PWDN -1  // power down is NC
#define CAM_PIN_RESET -1 // reset pin is NC, software reset used instead
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

static SemaphoreHandle_t cam_mutex;
int file_index = 0;

// Recording state
static FILE *record_file = NULL;
static bool is_recording = false;

static esp_err_t init_sdcard(void)
{
    esp_err_t ret;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT | SDMMC_HOST_FLAG_4BIT; // use 4-bit bus
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = GPIO_NUM_15;
    slot_config.cmd = GPIO_NUM_7;
    slot_config.d0  = GPIO_NUM_16;
    slot_config.d1  = GPIO_NUM_17;
    slot_config.d2  = GPIO_NUM_5;
    slot_config.d3  = GPIO_NUM_6;
    slot_config.width = 4;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) 
    {
        ESP_LOGE("SD", "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

static esp_err_t append_frame_to_recording(const uint8_t *buf, size_t len)
{
    if (!is_recording || !record_file)
    {
        return ESP_OK;
    }

    // Write raw JPEG frame directly into the MJPEG stream.
    // Many MJPEG readers can parse concatenated JPEG images.
    size_t written = fwrite(buf, 1, len, record_file);
    if (written != len)
    {
        ESP_LOGE("SD", "Failed to write MJPEG frame");
        fclose(record_file);
        record_file = NULL;
        is_recording = false;
        return ESP_FAIL;
    }

    fflush(record_file);
    return ESP_OK;
}

static esp_err_t init_camera(camera_resolution_t resolution, uint8_t jpeg_quality)
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
                                     .frame_size = resolution,

                                     .jpeg_quality = jpeg_quality, // 0-63, lower number = higher quality
                                     .fb_count = 2,
                                     .grab_mode = CAMERA_GRAB_LATEST};
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_vflip(sensor, true);
    sensor->set_hmirror(sensor, true);

    cam_mutex = xSemaphoreCreateMutex();

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

    xSemaphoreTake(cam_mutex, portMAX_DELAY);
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

    if (is_recording)
    {
        append_frame_to_recording(fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
    xSemaphoreGive(cam_mutex);

    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = (fr_end - last_frame) / 1000;
    last_frame = fr_end;
    //ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)", (unsigned int)(_jpg_buf_len / 1024), (unsigned int)frame_time, 1000.0 / (unsigned int)frame_time);

    last_frame = 0;    
}

static esp_err_t camera_apply_settings(framesize_t resolution, int jpeg_quality)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return ESP_FAIL;

    if (jpeg_quality < 0) jpeg_quality = 0;
    if (jpeg_quality > 63) jpeg_quality = 63;

    if (s->set_framesize(s, resolution) != 0) return ESP_FAIL;
    if (s->set_quality(s, jpeg_quality) != 0) return ESP_FAIL;

    return ESP_OK;
}

void handle_camera_telemetry(const void *payload)
{
    msg_header_t msg_header = {
        .msg_type = MSG_CAMERA,
        .payload_len = _jpg_buf_len,
    };

    send_message(msg_header, _jpg_buf);
}

void handle_cfg_camera(const void *payload)
{
    msg_cfg_camera_t* msg = (msg_cfg_camera_t*)payload;

    xSemaphoreTake(cam_mutex, portMAX_DELAY);
    camera_apply_settings(msg->camera_resolution, msg->compression_rate);
    xSemaphoreGive(cam_mutex);
}

void handle_camera_start_recording(const void *payload)
{
    char path[64];

    // If already recording, close current file and start a new one
    if (record_file)
    {
        fclose(record_file);
        record_file = NULL;
    }

    snprintf(path, sizeof(path), "/sdcard/%d.mjpeg", file_index++);
    record_file = fopen(path, "wb");
    if (!record_file)
    {
        ESP_LOGE("SD", "Failed to open recording file: %s", path);
        is_recording = false;
    }

    is_recording = true;
    ESP_LOGI("SD", "Started recording: %s", path);
}

void handle_camera_stop_recording(const void *payload)
{
    if (record_file)
    {
        fflush(record_file);
        fclose(record_file);
        record_file = NULL;
    }

    if (is_recording)
    {
        ESP_LOGI("SD", "Stopped recording");
    }

    is_recording = false;
}

#endif