
#include "driver/rmt_tx.h"
#include "led_strip/led_strip_encoder.h"

// LED pins and params
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 4

// Camera pins
#define CAM_PIN_PWDN -1  //power down is not used
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

static uint8_t led_strip_pixels[4 * 3];

rmt_channel_handle_t led_chan = NULL;
rmt_tx_channel_config_t tx_chan_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
    .gpio_num = RMT_LED_STRIP_GPIO_NUM,
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