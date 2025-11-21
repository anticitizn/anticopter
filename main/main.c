#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>
#include <nvs_flash.h>

static const char *TAG = "ANTICOPTER";

#include "camera.h"
#include "connect_wifi.h"
#include "http.h"
#include "imu.h"
#include "led.h"
#include "pwm_control.h"
#include "udp.h"
// #include "lsm6ds33/lsm6ds33.h"


void control_sensor_loop()
{
    while (true)
    {
        cam_take_picture();
        motors_tick();
        poll_lsm6ds3();
    }
}

void app_main()
{
    //lsm6ds3_read_data_polling();

    init_leds();

    set_led(0, 50, 0, 0);
    set_led(1, 50, 0, 0);
    set_led(2, 50, 0, 0);
    set_led(3, 50, 0, 0);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

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

    wifi_init_softap();

    err = init_camera();
    err = init_sdcard();
    if (err != ESP_OK)
    {
        printf("err: %s\n", esp_err_to_name(err));
        return;
    }

    init_lsm6ds3();

    xTaskCreatePinnedToCore(control_sensor_loop, "control_sensor_loop", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL, 1);
    ESP_LOGI(TAG, "Anticopter software is up and running\n");
    
}