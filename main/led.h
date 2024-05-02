
#include "esp_log.h"
#include "driver/rmt_tx.h"

#include "globals.h"

void set_led(int num, int r, int g, int b)
{
    led_strip_pixels[num * 3] = g;
    led_strip_pixels[num * 3 + 1] = r;
    led_strip_pixels[num * 3 + 2] = b;

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}