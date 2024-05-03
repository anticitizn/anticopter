
#ifndef ANTICOPTER_LED
#define ANTICOPTER_LED

#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_tx.h"
#include "led_strip/led_strip_encoder.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

static uint8_t led_strip_pixels[4 * 3];

rmt_channel_handle_t led_chan = NULL;
rmt_tx_channel_config_t tx_chan_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
    .gpio_num = 4, // GPIO 4
    .mem_block_symbols = 64, // increase the block size can make the LED less flickering
    .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
    .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
};

rmt_encoder_handle_t led_encoder = NULL;
led_strip_encoder_config_t encoder_config = {
    .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
};
rmt_transmit_config_t tx_config = {
    .loop_count = 0, // no transfer loop
};

void init_leds()
{
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}

void set_led(int num, int r, int g, int b)
{
    led_strip_pixels[num * 3] = g;
    led_strip_pixels[num * 3 + 1] = r;
    led_strip_pixels[num * 3 + 2] = b;

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

#endif