
#ifndef ANTICOPTER_IMU
#define ANTICOPTER_IMU

/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/i2c.h"
#include "esp_log.h"
#include "lsm6dsr/lsm6dsr_reg.h"
#include <stdio.h>

#define I2C_MASTER_SCL_IO 1 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 2 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define LSM6DSR_SENSOR_ADDR 0x6B

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
i2c_port_t i2cPort = I2C_MASTER_NUM;

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSR_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, (uint8_t *)bufp, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSR_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSR_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, bufp, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len) 
{
    return; 
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms) 
{
    vTaskDelay(ms / portTICK_PERIOD_MS); 
}

void lsm6dsr_read_data_polling(void)
{
    stmdev_ctx_t dev_ctx;
    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &i2cPort;
    /* Init test platform */
    platform_init();
    /* Wait sensor boot time */
    platform_delay(10);
    /* Check device ID */
    lsm6dsr_device_id_get(&dev_ctx, &whoamI);

    if (whoamI != LSM6DSR_ID)
        while (1)
            ;

    /* Restore default configuration */
    lsm6dsr_reset_set(&dev_ctx, PROPERTY_ENABLE);

    do
    {
        lsm6dsr_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* Disable I3C interface */
    lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);

    /* Enable Block Data Update */
    lsm6dsr_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set Output Data Rate */
    lsm6dsr_xl_data_rate_set(&dev_ctx, LSM6DSR_XL_ODR_104Hz);
    lsm6dsr_gy_data_rate_set(&dev_ctx, LSM6DSR_GY_ODR_104Hz);

    /* Set full scale */
    lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);
    lsm6dsr_gy_full_scale_set(&dev_ctx, LSM6DSR_1000dps);

    /* Configure filtering chain(No aux interface)
     * Accelerometer - LPF1 + LPF2 path
     */
    lsm6dsr_xl_hp_path_on_out_set(&dev_ctx, LSM6DSR_LP_ODR_DIV_100);
    lsm6dsr_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

    /* Read samples in polling mode */
    while (1)
    {
        uint8_t reg;
        /* Read output only if new xl value is available */
        lsm6dsr_xl_flag_data_ready_get(&dev_ctx, &reg);

        if (reg)
        {
            /* Read acceleration field data */
            memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
            lsm6dsr_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
            acceleration_mg[0] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]);
            acceleration_mg[1] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]);
            acceleration_mg[2] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);
            //printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
        }

        lsm6dsr_gy_flag_data_ready_get(&dev_ctx, &reg);

        if (reg)
        {
            /* Read angular rate field data */
            memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
            lsm6dsr_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
            angular_rate_mdps[0] = lsm6dsr_from_fs1000dps_to_mdps(data_raw_angular_rate[0]);
            angular_rate_mdps[1] = lsm6dsr_from_fs1000dps_to_mdps(data_raw_angular_rate[1]);
            angular_rate_mdps[2] = lsm6dsr_from_fs1000dps_to_mdps(data_raw_angular_rate[2]);
            //printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
        }

        lsm6dsr_temp_flag_data_ready_get(&dev_ctx, &reg);

        if (reg)
        {
            /* Read temperature data */
            memset(&data_raw_temperature, 0x00, sizeof(int16_t));
            lsm6dsr_temperature_raw_get(&dev_ctx, &data_raw_temperature);
            temperature_degC = lsm6dsr_from_lsb_to_celsius(data_raw_temperature);
            //printf("Temperature [degC]:%6.2f\r\n", temperature_degC);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
        }
    }
}

#endif
