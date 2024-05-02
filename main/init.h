
#include "esp_log.h"
#include "driver/rmt_tx.h"

#include "globals.h"

void init_leds()
{
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}