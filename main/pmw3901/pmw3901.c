
#include "pmw3901.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_rom_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/* ---------------------------------------------------------
 * Configuration
 * --------------------------------------------------------- */

#define PMW3901_CS_GPIO       GPIO_NUM_0
#define PMW3901_MISO_GPIO     GPIO_NUM_38
#define PMW3901_MOSI_GPIO     GPIO_NUM_44
#define PMW3901_CLK_GPIO      GPIO_NUM_43

#define PMW3901_SPI_HOST      SPI2_HOST

#define PMW3901_SPI_FREQ_HZ   500000

#define PMW3901_PRODUCT_ID    0x49
#define PMW3901_INVERSE_ID    0xB6


/* ---------------------------------------------------------
 * Registers
 * --------------------------------------------------------- */

#define PMW3901_REG_PRODUCT_ID       0x00
#define PMW3901_REG_MOTION           0x02
#define PMW3901_REG_DELTA_X_L        0x03
#define PMW3901_REG_DELTA_X_H        0x04
#define PMW3901_REG_DELTA_Y_L        0x05
#define PMW3901_REG_DELTA_Y_H        0x06

#define PMW3901_REG_MOTION_BURST     0x16

#define PMW3901_REG_POWER_UP_RESET   0x3A
#define PMW3901_REG_INVERSE_ID       0x5F


/* ---------------------------------------------------------
 * State
 * --------------------------------------------------------- */

static const char *PMW3901_TAG = "PMW3901";

static spi_device_handle_t pmw3901_spi = NULL;

static bool pmw3901_initialized = false;


/* ---------------------------------------------------------
 * Helpers
 * --------------------------------------------------------- */

static inline void pmw3901_delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}


static inline void pmw3901_cs_low(void)
{
    gpio_set_level(PMW3901_CS_GPIO, 0);
}


static inline void pmw3901_cs_high(void)
{
    gpio_set_level(PMW3901_CS_GPIO, 1);
}


/*
 * Transfer one byte and optionally return the received byte.
 *
 * PMW3901 is full-duplex SPI, so every transmitted byte
 * simultaneously clocks in one received byte.
 */
static esp_err_t pmw3901_transfer_byte(
    uint8_t tx,
    uint8_t *rx)
{
    spi_transaction_t transaction = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA |
                 SPI_TRANS_USE_RXDATA,
    };

    transaction.tx_data[0] = tx;

    esp_err_t err =
        spi_device_polling_transmit(
            pmw3901_spi,
            &transaction);

    if ((err == ESP_OK) && (rx != NULL)) {
        *rx = transaction.rx_data[0];
    }

    return err;
}


/* ---------------------------------------------------------
 * Register write
 * --------------------------------------------------------- */
static esp_err_t pmw3901_register_write(
    uint8_t reg,
    uint8_t value)
{
    reg |= 0x80;

    esp_err_t err =
        spi_device_acquire_bus(
            pmw3901_spi,
            portMAX_DELAY);

    if (err != ESP_OK) {
        return err;
    }

    pmw3901_cs_low();

    esp_rom_delay_us(50);

    err = pmw3901_transfer_byte(
        reg,
        NULL);

    if (err == ESP_OK) {
        esp_rom_delay_us(50);

        err = pmw3901_transfer_byte(
            value,
            NULL);
    }

    esp_rom_delay_us(50);

    pmw3901_cs_high();

    spi_device_release_bus(
        pmw3901_spi);

    esp_rom_delay_us(200);

    return err;
}


/* ---------------------------------------------------------
 * Register read
 * --------------------------------------------------------- */
static esp_err_t pmw3901_register_read(
    uint8_t reg,
    uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    reg &= 0x7F;

    esp_err_t err =
        spi_device_acquire_bus(
            pmw3901_spi,
            portMAX_DELAY);

    if (err != ESP_OK) {
        return err;
    }

    pmw3901_cs_low();

    esp_rom_delay_us(50);

    err = pmw3901_transfer_byte(
        reg,
        NULL);

    if (err == ESP_OK) {
        esp_rom_delay_us(500);

        err = pmw3901_transfer_byte(
            0x00,
            value);
    }

    esp_rom_delay_us(50);

    pmw3901_cs_high();

    spi_device_release_bus(
        pmw3901_spi);

    esp_rom_delay_us(200);

    return err;
}


/* ---------------------------------------------------------
 * Performance initialization registers
 *
 * Sequence retained from the Bitcraze PMW3901 driver.
 * --------------------------------------------------------- */

static esp_err_t pmw3901_init_registers(void)
{
#define WRITE(REG, VALUE) do {                         \
    esp_err_t e = pmw3901_register_write(REG, VALUE); \
    if (e != ESP_OK) return e;                         \
} while (0)

#define READ(REG, PTR) do {                    \
    esp_err_t e = pmw3901_register_read(REG, PTR); \
    if (e != ESP_OK) return e;                 \
} while (0)

    uint8_t v = 0;
    uint8_t c1 = 0;
    uint8_t c2 = 0;

    /* PixArt demo kit V3.20 calibration section */
    WRITE(0x7F, 0x00);
    WRITE(0x55, 0x01);
    WRITE(0x50, 0x07);

    WRITE(0x7F, 0x0E);
    WRITE(0x43, 0x10);

    READ(0x67, &v);

    if (v & 0x80) {
        WRITE(0x48, 0x04);
    } else {
        WRITE(0x48, 0x02);
    }

    WRITE(0x7F, 0x00);
    WRITE(0x51, 0x7B);
    WRITE(0x50, 0x00);
    WRITE(0x55, 0x00);

    WRITE(0x7F, 0x0E);

    READ(0x73, &v);

    if (v == 0) {
        READ(0x70, &c1);

        if (c1 <= 28) {
            c1 += 14;
        } else {
            c1 += 11;
        }

        if (c1 > 0x3F) {
            c1 = 0x3F;
        }

        READ(0x71, &c2);

        c2 = ((uint16_t)c2 * 45) / 100;

        WRITE(0x7F, 0x00);
        WRITE(0x61, 0xAD);
        WRITE(0x51, 0x70);

        WRITE(0x7F, 0x0E);
        WRITE(0x70, c1);
        WRITE(0x71, c2);
    }

    /* Existing fixed sequence follows */
    WRITE(0x7F, 0x00);
    WRITE(0x61, 0xAD);

    WRITE(0x7F, 0x03);
    WRITE(0x40, 0x00);

    WRITE(0x7F, 0x05);
    WRITE(0x41, 0xB3);
    WRITE(0x43, 0xF1);
    WRITE(0x45, 0x14);
    WRITE(0x5B, 0x32);
    WRITE(0x5F, 0x34);
    WRITE(0x7B, 0x08);

    WRITE(0x7F, 0x06);
    WRITE(0x44, 0x1B);
    WRITE(0x40, 0xBF);
    WRITE(0x4E, 0x3F);

    WRITE(0x7F, 0x08);
    WRITE(0x65, 0x20);
    WRITE(0x6A, 0x18);

    WRITE(0x7F, 0x09);
    WRITE(0x4F, 0xAF);
    WRITE(0x5F, 0x40);
    WRITE(0x48, 0x80);
    WRITE(0x49, 0x80);
    WRITE(0x57, 0x77);
    WRITE(0x60, 0x78);
    WRITE(0x61, 0x78);
    WRITE(0x62, 0x08);
    WRITE(0x63, 0x50);

    WRITE(0x7F, 0x0A);
    WRITE(0x45, 0x60);

    WRITE(0x7F, 0x00);
    WRITE(0x4D, 0x11);
    WRITE(0x55, 0x80);

    /* Note: PX4 uses 0x21, not 0x1F */
    WRITE(0x74, 0x21);

    WRITE(0x75, 0x1F);
    WRITE(0x4A, 0x78);
    WRITE(0x4B, 0x78);
    WRITE(0x44, 0x08);
    WRITE(0x45, 0x50);
    WRITE(0x64, 0xFF);
    WRITE(0x65, 0x1F);

    WRITE(0x7F, 0x14);
    WRITE(0x65, 0x67);
    WRITE(0x66, 0x08);
    WRITE(0x63, 0x70);

    WRITE(0x7F, 0x15);
    WRITE(0x48, 0x48);

    WRITE(0x7F, 0x07);
    WRITE(0x41, 0x0D);
    WRITE(0x43, 0x14);
    WRITE(0x4B, 0x0E);
    WRITE(0x45, 0x0F);
    WRITE(0x44, 0x42);
    WRITE(0x4C, 0x80);

    WRITE(0x7F, 0x10);
    WRITE(0x5B, 0x02);

    WRITE(0x7F, 0x07);
    WRITE(0x40, 0x41);
    WRITE(0x70, 0x00);

    vTaskDelay(pdMS_TO_TICKS(10));

    WRITE(0x32, 0x44);

    WRITE(0x7F, 0x07);
    WRITE(0x40, 0x40);

    WRITE(0x7F, 0x06);
    WRITE(0x62, 0xF0);
    WRITE(0x63, 0x00);

    WRITE(0x7F, 0x0D);
    WRITE(0x48, 0xC0);
    WRITE(0x6F, 0xD5);

    WRITE(0x7F, 0x00);
    WRITE(0x5B, 0xA0);
    WRITE(0x4E, 0xA8);
    WRITE(0x5A, 0x50);
    WRITE(0x40, 0x80);

#undef READ
#undef WRITE

    return ESP_OK;
}


/* ---------------------------------------------------------
 * SPI initialization
 * --------------------------------------------------------- */

static esp_err_t pmw3901_spi_init(void)
{
    /*
     * CS is controlled manually.
     *
     * The PMW3901 requires delays between address/data phases
     * while CS remains low, so hardware CS is intentionally
     * disabled with spics_io_num = -1.
     */
    gpio_config_t cs_config = {
        .pin_bit_mask = 1ULL << PMW3901_CS_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&cs_config);

    if (err != ESP_OK) {
        return err;
    }

    pmw3901_cs_high();


    spi_bus_config_t bus_config = {
        .mosi_io_num = PMW3901_MOSI_GPIO,
        .miso_io_num = PMW3901_MISO_GPIO,
        .sclk_io_num = PMW3901_CLK_GPIO,

        .quadwp_io_num = -1,
        .quadhd_io_num = -1,

        /*
         * Motion burst is only 12 bytes, so this is plenty.
         */
        .max_transfer_sz = 32,
    };


    err = spi_bus_initialize(
        PMW3901_SPI_HOST,
        &bus_config,
        SPI_DMA_DISABLED);

    if (err != ESP_OK) {
        ESP_LOGE(
            PMW3901_TAG,
            "spi_bus_initialize failed: %s",
            esp_err_to_name(err));

        return err;
    }


    spi_device_interface_config_t device_config = {
        /*
         * PMW3901 uses SPI mode 3:
         *
         * CPOL = 1
         * CPHA = 1
         */
        .mode = 3,

        .clock_speed_hz = PMW3901_SPI_FREQ_HZ,

        /*
         * Manual CS control.
         */
        .spics_io_num = -1,

        .queue_size = 1,
    };


    err = spi_bus_add_device(
        PMW3901_SPI_HOST,
        &device_config,
        &pmw3901_spi);

    if (err != ESP_OK) {

        ESP_LOGE(
            PMW3901_TAG,
            "spi_bus_add_device failed: %s",
            esp_err_to_name(err));

        spi_bus_free(PMW3901_SPI_HOST);

        pmw3901_spi = NULL;

        return err;
    }


    return ESP_OK;
}


/* ---------------------------------------------------------
 * PMW3901 initialization
 * --------------------------------------------------------- */
esp_err_t pmw3901_init(void)
{
    if (pmw3901_initialized) {
        return ESP_OK;
    }

    /* -----------------------------------------------------
     * Initialize SPI
     * ----------------------------------------------------- */

    esp_err_t err = pmw3901_spi_init();

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Allow sensor power to stabilize.
     */
    vTaskDelay(pdMS_TO_TICKS(40));


    /* -----------------------------------------------------
     * Reset SPI interface
     * ----------------------------------------------------- */

    /*
     * PMW3901 startup sequence:
     * toggle NCS to reset/synchronize the SPI interface.
     */
    pmw3901_cs_high();
    vTaskDelay(pdMS_TO_TICKS(1));

    pmw3901_cs_low();
    vTaskDelay(pdMS_TO_TICKS(1));

    pmw3901_cs_high();
    vTaskDelay(pdMS_TO_TICKS(1));


    /* -----------------------------------------------------
     * Power-on reset
     * ----------------------------------------------------- */

    /*
     * IMPORTANT:
     *
     * The power-on reset must happen BEFORE checking
     * Product_ID / Inverse_Product_ID.
     */
    err = pmw3901_register_write(
        PMW3901_REG_POWER_UP_RESET,
        0x5A);

    if (err != ESP_OK) {
        ESP_LOGE(
            PMW3901_TAG,
            "Power-on reset failed: %s",
            esp_err_to_name(err));

        return err;
    }

    /*
     * PMW3901 requires approximately 5 ms after reset.
     */
    vTaskDelay(pdMS_TO_TICKS(5));


    /* -----------------------------------------------------
     * Verify sensor identity
     * ----------------------------------------------------- */

    uint8_t chip_id = 0;
    uint8_t inverse_chip_id = 0;

    err = pmw3901_register_read(
        PMW3901_REG_PRODUCT_ID,
        &chip_id);

    if (err != ESP_OK) {
        ESP_LOGE(
            PMW3901_TAG,
            "Failed reading product ID: %s",
            esp_err_to_name(err));

        return err;
    }

    err = pmw3901_register_read(
        PMW3901_REG_INVERSE_ID,
        &inverse_chip_id);

    if (err != ESP_OK) {
        ESP_LOGE(
            PMW3901_TAG,
            "Failed reading inverse product ID: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        PMW3901_TAG,
        "Chip ID: 0x%02X, inverse ID: 0x%02X",
        chip_id,
        inverse_chip_id);

    if ((chip_id != PMW3901_PRODUCT_ID) ||
        (inverse_chip_id != PMW3901_INVERSE_ID)) {

        ESP_LOGE(
            PMW3901_TAG,
            "Unexpected PMW3901 ID "
            "(expected 0x%02X:0x%02X)",
            PMW3901_PRODUCT_ID,
            PMW3901_INVERSE_ID);

        return ESP_ERR_NOT_FOUND;
    }


    /* -----------------------------------------------------
     * Clear startup motion registers
     * ----------------------------------------------------- */

    /*
     * After power-on reset the motion and delta registers
     * must be read once before applying the configuration
     * sequence.
     */
    uint8_t dummy = 0;

    err = pmw3901_register_read(
        PMW3901_REG_MOTION,
        &dummy);

    if (err != ESP_OK) {
        return err;
    }

    err = pmw3901_register_read(
        PMW3901_REG_DELTA_X_L,
        &dummy);

    if (err != ESP_OK) {
        return err;
    }

    err = pmw3901_register_read(
        PMW3901_REG_DELTA_X_H,
        &dummy);

    if (err != ESP_OK) {
        return err;
    }

    err = pmw3901_register_read(
        PMW3901_REG_DELTA_Y_L,
        &dummy);

    if (err != ESP_OK) {
        return err;
    }

    err = pmw3901_register_read(
        PMW3901_REG_DELTA_Y_H,
        &dummy);

    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(1));


    /* -----------------------------------------------------
     * Load PMW3901 performance configuration
     * ----------------------------------------------------- */

    err = pmw3901_init_registers();

    if (err != ESP_OK) {
        ESP_LOGE(
            PMW3901_TAG,
            "Performance register initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }


    /* -----------------------------------------------------
     * Initialization complete
     * ----------------------------------------------------- */

    pmw3901_initialized = true;

    ESP_LOGI(
        PMW3901_TAG,
        "Initialized successfully");

    return ESP_OK;
}


/* ---------------------------------------------------------
 * Motion burst
 * --------------------------------------------------------- */

esp_err_t pmw3901_read_motion(
    motionBurst_t *motion)
{
    if (motion == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!pmw3901_initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    /*
     * Motion burst address.
     *
     * MSB remains 0 because this is a read operation.
     */
    uint8_t address =
        PMW3901_REG_MOTION_BURST & 0x7F;


    pmw3901_cs_low();

    pmw3901_delay_us(50);


    esp_err_t err =
        pmw3901_transfer_byte(
            address,
            NULL);

    if (err != ESP_OK) {
        pmw3901_cs_high();
        return err;
    }


    pmw3901_delay_us(50);


    /*
     * Clock out the complete burst.
     */
    uint8_t dummy_tx[sizeof(motionBurst_t)] = {0};
    uint8_t rx_data[sizeof(motionBurst_t)] = {0};

    spi_transaction_t transaction = {
        .length = sizeof(motionBurst_t) * 8,
        .tx_buffer = dummy_tx,
        .rx_buffer = rx_data,
    };


    err = spi_device_polling_transmit(
        pmw3901_spi,
        &transaction);


    pmw3901_delay_us(50);

    pmw3901_cs_high();

    pmw3901_delay_us(50);


    if (err != ESP_OK) {
        return err;
    }


    memcpy(
        motion,
        rx_data,
        sizeof(motionBurst_t));


    /*
     * The PMW3901 sends shutter MSB first.
     *
     * The ESP32 is little endian, so swap the two bytes.
     */
    motion->shutter =
        (uint16_t)(
            (motion->shutter >> 8) |
            (motion->shutter << 8));


    return ESP_OK;
}


/* ---------------------------------------------------------
 * State
 * --------------------------------------------------------- */

bool pmw3901_is_initialized(void)
{
    return pmw3901_initialized;
}


motionBurst_t pmw3901_poll()
{
    motionBurst_t motion;

      if (pmw3901_read_motion(&motion) == ESP_OK) {
          ESP_LOGI(
              "FLOW",
              "dx=%d dy=%d squal=%u motion=%u",
              motion.deltaX,
              motion.deltaY,
              motion.squal,
              motion.motionOccurred
          );
      }

    return motion;
}

void pmw3901_debug_motion_registers(void)
{
    uint8_t motion = 0;
    uint8_t observation = 0;
    uint8_t dx_l = 0;
    uint8_t dx_h = 0;
    uint8_t dy_l = 0;
    uint8_t dy_h = 0;
    uint8_t squal = 0;

    pmw3901_register_read(0x02, &motion);
    pmw3901_register_read(0x03, &dx_l);
    pmw3901_register_read(0x04, &dx_h);
    pmw3901_register_read(0x05, &dy_l);
    pmw3901_register_read(0x06, &dy_h);
    pmw3901_register_read(0x07, &squal);
    pmw3901_register_read(0x15, &observation);

    int16_t dx = (int16_t)(
        ((uint16_t)dx_h << 8) | dx_l);

    int16_t dy = (int16_t)(
        ((uint16_t)dy_h << 8) | dy_l);

    ESP_LOGI(
        "PMWDBG",
        "motion=%02X obs=%02X dx=%d dy=%d squal=%u",
        motion,
        observation,
        dx,
        dy,
        squal);
}

void pmw3901_debug_image(void)
{
    uint8_t motion;
    uint8_t squal;
    uint8_t sum;
    uint8_t max;
    uint8_t min;
    uint8_t shutter_l;
    uint8_t shutter_h;
    uint8_t observation;

    pmw3901_register_write(0x7F, 0x00);

    pmw3901_register_read(0x02, &motion);
    pmw3901_register_read(0x07, &squal);
    pmw3901_register_read(0x08, &sum);
    pmw3901_register_read(0x09, &max);
    pmw3901_register_read(0x0A, &min);
    pmw3901_register_read(0x0B, &shutter_l);
    pmw3901_register_read(0x0C, &shutter_h);
    pmw3901_register_read(0x15, &observation);

    uint16_t shutter =
        ((uint16_t)shutter_h << 8) | shutter_l;

    ESP_LOGI(
        "PMWIMG",
        "mot=%02X obs=%02X "
        "squal=%u sum=%u min=%u max=%u shutter=%u",
        motion,
        observation,
        squal,
        sum,
        min,
        max,
        shutter
    );
}

#define PMW3901_FRAME_W 35
#define PMW3901_FRAME_H 35
#define PMW3901_FRAME_PIXELS (PMW3901_FRAME_W * PMW3901_FRAME_H)

esp_err_t pmw3901_capture_frame(uint8_t *frame)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;

#define WR(r, v) do {                         \
    err = pmw3901_register_write((r), (v));   \
    if (err != ESP_OK) return err;            \
} while (0)

#define RD(r, p) do {                         \
    err = pmw3901_register_read((r), (p));    \
    if (err != ESP_OK) return err;            \
} while (0)

    /*
     * Enter raw frame capture mode.
     *
     * This sequence is taken from the Bitcraze/Pimoroni-derived
     * PMW3901 framebuffer implementation.
     */
    WR(0x7F, 0x07);
    WR(0x4C, 0x00);

    WR(0x7F, 0x08);
    WR(0x6A, 0x38);

    WR(0x7F, 0x00);
    WR(0x55, 0x04);
    WR(0x40, 0x80);
    WR(0x4D, 0x11);

    vTaskDelay(pdMS_TO_TICKS(10));

    WR(0x7F, 0x00);
    WR(0x58, 0xFF);

    /*
     * Wait for raw-data grab to become ready.
     *
     * Register 0x59 contains status in bits 7:6.
     */
    uint8_t status = 0;

    int timeout_ms = 5000;

    while (timeout_ms-- > 0) {
        RD(0x59, &status);

        if (status & 0xC0) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if ((status & 0xC0) == 0) {
        ESP_LOGE(
            "PMWFRAME",
            "Frame capture ready timeout, status=%02X",
            status);

        return ESP_ERR_TIMEOUT;
    }

    /*
     * Start retrieving raw pixels.
     */
    WR(0x58, 0x00);

    memset(frame, 0, PMW3901_FRAME_PIXELS);

    int pixel = 0;
    int read_timeout = 100000;

    while ((pixel < PMW3901_FRAME_PIXELS) &&
           (read_timeout-- > 0)) {

        uint8_t data;

        RD(0x58, &data);

        /*
         * Bits 7:6 tell us what part of the pixel this is.
         *
         * 01xxxxxx:
         *   bits 5:0 contain pixel bits 7:2
         *
         * 10xxxxxx:
         *   bits 3:2 contain pixel bits 1:0
         */
        switch (data & 0xC0) {
            case 0x40:
                frame[pixel] =
                    (frame[pixel] & 0x03) |
                    ((data & 0x3F) << 2);
                break;

            case 0x80:
                frame[pixel] =
                    (frame[pixel] & 0xFC) |
                    ((data & 0x0C) >> 2);

                pixel++;
                break;

            default:
                /*
                 * Ignore values that don't carry pixel data.
                 */
                break;
        }
    }

    if (pixel != PMW3901_FRAME_PIXELS) {
        ESP_LOGE(
            "PMWFRAME",
            "Frame capture incomplete: %d/%d pixels",
            pixel,
            PMW3901_FRAME_PIXELS);

        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(
        "PMWFRAME",
        "Captured %d pixels",
        pixel);

#undef WR
#undef RD

    return ESP_OK;
}

void pmw3901_print_frame(void)
{
    static uint8_t frame[PMW3901_FRAME_PIXELS];

    esp_err_t err = pmw3901_capture_frame(frame);

    if (err != ESP_OK) {
        ESP_LOGE(
            "PMWFRAME",
            "capture failed: %s",
            esp_err_to_name(err));
        return;
    }

    static const char chars[] = " .:-=+*#%@";
    const int levels = sizeof(chars) - 2;

    printf("\n");

    for (int y = 0; y < PMW3901_FRAME_H; y++) {
        for (int x = 0; x < PMW3901_FRAME_W; x++) {

            uint8_t p =
                frame[y * PMW3901_FRAME_W + x];

            int i =
                ((int)p * levels) / 255;

            putchar(chars[i]);
            putchar(chars[i]);  // compensate terminal aspect ratio
        }

        putchar('\n');
    }

    printf("\n");
}
