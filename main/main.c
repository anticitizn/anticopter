#include <esp_system.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "ANTICOPTER";

#include "connect_wifi.h"
#include "pwm_control.h"
#include "imu.h"
#include "led.h"
#include "http.h"
#include "led.h"
#include "camera.h"
#include "udp.h"


void app_main()
{
    //lsm6dsr_read_data_polling();

    init_leds();

    printf("Starting leds!");

    set_led(0, 50, 0, 0);
    set_led(1, 50, 0, 0);
    set_led(2, 50, 0, 0);
    set_led(3, 50, 0, 0);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    printf("Stopping leds!");

    set_led(0, 0, 0, 0);
    set_led(1, 0, 0, 0);
    set_led(2, 0, 0, 0);
    set_led(3, 0, 0, 0);

    setup_pwm();
    motors_check();

    esp_err_t err;

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    connect_wifi();

    if (wifi_connect_status)
    {
        err = init_camera();
        if (err != ESP_OK)
        {
            printf("err: %s\n", esp_err_to_name(err));
            return;
        }
        setup_server();
        xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
        ESP_LOGI(TAG, "ESP32 CAM Web Server is up and running\n");
    }
    else
        ESP_LOGI(TAG, "Failed to connected with Wi-Fi, check your network Credentials\n");
}