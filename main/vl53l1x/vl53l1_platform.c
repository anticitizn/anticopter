/**
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 */

#include "vl53l1_platform.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdint.h>


/* ---------------------------------------------------------
 * I2C Configuration
 *
 * Same configuration previously used by the IMU.
 * --------------------------------------------------------- */
#define I2C_MASTER_SCL_IO          1
#define I2C_MASTER_SDA_IO          2
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0

#define I2C_MASTER_TIMEOUT_MS      1000


/* ---------------------------------------------------------
 * Platform initialization
 * --------------------------------------------------------- */

int8_t VL53L1_PlatformInit(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,

        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,

        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,

        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err;

    err = i2c_param_config(I2C_MASTER_NUM, &conf);

    if (err != ESP_OK) {
        return -1;
    }

    err = i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE,
        0
    );

    if (err != ESP_OK) {
        return -1;
    }

    return 0;
}


/* ---------------------------------------------------------
 * Helpers
 * --------------------------------------------------------- */

/*
 * ST ULD uses an 8-bit I2C address:
 *
 *     0x52 = write address
 *     0x53 = read address
 *
 * ESP-IDF's transaction API below starts with a 7-bit address
 * and adds the R/W bit itself, so:
 *
 *     0x52 >> 1 = 0x29
 */
static inline uint8_t vl53l1_addr_7bit(uint16_t dev)
{
    return (uint8_t)(dev >> 1);
}


static inline int8_t vl53l1_status(esp_err_t err)
{
    return (err == ESP_OK) ? 0 : -1;
}


/* ---------------------------------------------------------
 * Multi-byte write
 *
 * START
 * DEVICE + WRITE
 * REGISTER MSB
 * REGISTER LSB
 * DATA...
 * STOP
 * --------------------------------------------------------- */

int8_t VL53L1_WriteMulti(
    uint16_t dev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count)
{
    uint8_t addr = vl53l1_addr_7bit(dev);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return -1;
    }

    esp_err_t err = ESP_OK;

    err = i2c_master_start(cmd);

    if (err == ESP_OK) {
        err = i2c_master_write_byte(
            cmd,
            (addr << 1) | I2C_MASTER_WRITE,
            true
        );
    }

    /*
     * VL53L1X has 16-bit register addresses.
     * Send MSB first.
     */
    if (err == ESP_OK) {
        err = i2c_master_write_byte(
            cmd,
            (uint8_t)(index >> 8),
            true
        );
    }

    if (err == ESP_OK) {
        err = i2c_master_write_byte(
            cmd,
            (uint8_t)(index & 0xFF),
            true
        );
    }

    if ((err == ESP_OK) && (count > 0)) {
        err = i2c_master_write(
            cmd,
            pdata,
            count,
            true
        );
    }

    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }

    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(
            I2C_MASTER_NUM,
            cmd,
            pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
        );
    }

    i2c_cmd_link_delete(cmd);

    return vl53l1_status(err);
}


/* ---------------------------------------------------------
 * Multi-byte read
 *
 * START
 * DEVICE + WRITE
 * REGISTER MSB
 * REGISTER LSB
 * REPEATED START
 * DEVICE + READ
 * DATA...
 * STOP
 * --------------------------------------------------------- */

int8_t VL53L1_ReadMulti(
    uint16_t dev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count)
{
    if ((pdata == NULL) || (count == 0)) {
        return -1;
    }

    uint8_t addr = vl53l1_addr_7bit(dev);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return -1;
    }

    esp_err_t err = ESP_OK;

    err = i2c_master_start(cmd);

    if (err == ESP_OK) {
        err = i2c_master_write_byte(
            cmd,
            (addr << 1) | I2C_MASTER_WRITE,
            true
        );
    }

    if (err == ESP_OK) {
        err = i2c_master_write_byte(
            cmd,
            (uint8_t)(index >> 8),
            true
        );
    }

    if (err == ESP_OK) {
        err = i2c_master_write_byte(
            cmd,
            (uint8_t)(index & 0xFF),
            true
        );
    }

    /*
     * Repeated START.
     */
    if (err == ESP_OK) {
        err = i2c_master_start(cmd);
    }

    if (err == ESP_OK) {
        err = i2c_master_write_byte(
            cmd,
            (addr << 1) | I2C_MASTER_READ,
            true
        );
    }

    /*
     * ACK every byte except the last.
     * NACK the final byte.
     */
    if (err == ESP_OK) {

        if (count == 1) {

            err = i2c_master_read_byte(
                cmd,
                pdata,
                I2C_MASTER_NACK
            );

        } else {

            err = i2c_master_read(
                cmd,
                pdata,
                count - 1,
                I2C_MASTER_ACK
            );

            if (err == ESP_OK) {
                err = i2c_master_read_byte(
                    cmd,
                    &pdata[count - 1],
                    I2C_MASTER_NACK
                );
            }
        }
    }

    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }

    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(
            I2C_MASTER_NUM,
            cmd,
            pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
        );
    }

    i2c_cmd_link_delete(cmd);

    return vl53l1_status(err);
}


/* ---------------------------------------------------------
 * Basic register operations
 * --------------------------------------------------------- */

int8_t VL53L1_WrByte(
    uint16_t dev,
    uint16_t index,
    uint8_t data)
{
    return VL53L1_WriteMulti(
        dev,
        index,
        &data,
        1
    );
}


int8_t VL53L1_WrWord(
    uint16_t dev,
    uint16_t index,
    uint16_t data)
{
    uint8_t buffer[2];

    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0xFF);

    return VL53L1_WriteMulti(
        dev,
        index,
        buffer,
        2
    );
}


int8_t VL53L1_WrDWord(
    uint16_t dev,
    uint16_t index,
    uint32_t data)
{
    uint8_t buffer[4];

    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)(data >> 16);
    buffer[2] = (uint8_t)(data >> 8);
    buffer[3] = (uint8_t)data;

    return VL53L1_WriteMulti(
        dev,
        index,
        buffer,
        4
    );
}


int8_t VL53L1_RdByte(
    uint16_t dev,
    uint16_t index,
    uint8_t *data)
{
    return VL53L1_ReadMulti(
        dev,
        index,
        data,
        1
    );
}


int8_t VL53L1_RdWord(
    uint16_t dev,
    uint16_t index,
    uint16_t *data)
{
    if (data == NULL) {
        return -1;
    }

    uint8_t buffer[2];

    int8_t status = VL53L1_ReadMulti(
        dev,
        index,
        buffer,
        2
    );

    if (status != 0) {
        return status;
    }

    *data =
        ((uint16_t)buffer[0] << 8) |
        (uint16_t)buffer[1];

    return 0;
}


int8_t VL53L1_RdDWord(
    uint16_t dev,
    uint16_t index,
    uint32_t *data)
{
    if (data == NULL) {
        return -1;
    }

    uint8_t buffer[4];

    int8_t status = VL53L1_ReadMulti(
        dev,
        index,
        buffer,
        4
    );

    if (status != 0) {
        return status;
    }

    *data =
        ((uint32_t)buffer[0] << 24) |
        ((uint32_t)buffer[1] << 16) |
        ((uint32_t)buffer[2] << 8)  |
        ((uint32_t)buffer[3]);

    return 0;
}


/* ---------------------------------------------------------
 * Delay
 * --------------------------------------------------------- */

int8_t VL53L1_WaitMs(
    uint16_t dev,
    int32_t wait_ms)
{
    (void)dev;

    if (wait_ms <= 0) {
        return 0;
    }

    TickType_t ticks = pdMS_TO_TICKS(wait_ms);

    /*
     * Ensure a requested positive delay never gets rounded
     * down to zero ticks.
     */
    if (ticks == 0) {
        ticks = 1;
    }

    vTaskDelay(ticks);

    return 0;
}