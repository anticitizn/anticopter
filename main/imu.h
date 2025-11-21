#ifndef ANTICOPTER_IMU
#define ANTICOPTER_IMU

#include "driver/i2c.h"
#include "esp_log.h"
#include "lsm6ds3/lsm6ds3_reg.h"
#include "camera.h"
#include <stdio.h>
#include <math.h>

#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_SDA_IO 2
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define LSM6DS3_SENSOR_ADDR 0x6A

#define RAD_TO_DEG (180.0f / M_PI)
#define Kp_BASE  2.0f
#define Kp_BOOST 10.0f
#define Ki       0.005f

static float Kp_current = Kp_BASE;
static float ahrs_time  = 0.0f;

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float integralFB[3] = {0.0f, 0.0f, 0.0f};

static int16_t data_raw_acceleration[3] = {0.0f, 0.0f, 0.0f};
static int16_t data_raw_angular_rate[3] = {0.0f, 0.0f, 0.0f};

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float orientation_offset[3] = {0.0f, 0.0f, 0.0f};

static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static float orientation[3];

static int64_t last_time = 0;
static uint8_t whoamI, rst;

static uint8_t tx_buffer[1000];
static i2c_port_t i2cPort = I2C_MASTER_NUM;

static void platform_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE,
        0
    );
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
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
    i2c_master_write_byte(cmd, (LSM6DS3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS3_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, bufp, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return 0;
}

static void platform_delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

static void tx_com(uint8_t *buf, uint16_t len)
{
    return; // Keep disabled as in your original code
}


/*--------------------------------------------------------------------
   Mahony Filter
 --------------------------------------------------------------------*/
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm <= 0.0f) return;

    ax /= norm;
    ay /= norm;
    az /= norm;

    vx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    vy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    if (Ki > 0.0f)
    {
        integralFB[0] += Ki * ex * dt;
        integralFB[1] += Ki * ey * dt;
        integralFB[2] += Ki * ez * dt;

        gx += integralFB[0];
        gy += integralFB[1];
        gz += integralFB[2];
    }

    gx += Kp_current * ex;
    gy += Kp_current * ey;
    gz += Kp_current * ez;


    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qDot1 = -q[1] * gx - q[2] * gy - q[3] * gz;
    float qDot2 = q[0] * gx + q[2] * gz - q[3] * gy;
    float qDot3 = q[0] * gy - q[1] * gz + q[3] * gx;
    float qDot4 = q[0] * gz + q[1] * gy - q[2] * gx;

    q[0] += qDot1;
    q[1] += qDot2;
    q[2] += qDot3;
    q[3] += qDot4;

    norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
}

void estimate_position_orientation(float accel[3], float gyro[3], double dt)
{
    // Early-time Kp boost with linear ramp-down
    ahrs_time += (float)dt;

    if (ahrs_time < 0.5f) 
    {
        // Strong correction during first 0.5 s
        Kp_current = Kp_BOOST;
    } else if (ahrs_time < 1.0f) 
    {
        // Linearly ramp from Kp_BOOST down to Kp_BASE between 0.5–1.0 s
        float t = (ahrs_time - 0.5f) / 0.5f;  // 0 .. 1
        Kp_current = Kp_BOOST + t * (Kp_BASE - Kp_BOOST);
    } else 
    {
        // Normal operation
        Kp_current = Kp_BASE;
    }

    float gyro_rad[3];
    gyro_rad[0] = gyro[0] * M_PI / 180.0f;
    gyro_rad[1] = gyro[1] * M_PI / 180.0f;
    gyro_rad[2] = gyro[2] * M_PI / 180.0f;

    MahonyAHRSupdate(
        gyro_rad[0], gyro_rad[1], gyro_rad[2],
        accel[0], accel[1], accel[2],
        dt
    );

    float roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                        1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    float pitch = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
    float yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                       1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));

    orientation[0] = roll * RAD_TO_DEG;
    orientation[1] = pitch * RAD_TO_DEG;
    orientation[2] = yaw * RAD_TO_DEG;
}

static void calibrate_gyro(stmdev_ctx_t *dev_ctx)
{
    const int samples = 500;
    int32_t sum[3] = {0, 0, 0};
    int16_t raw[3];

    // let sensor settle
    platform_delay(200);  

    for (int i = 0; i < samples; i++)
    {
        uint8_t drdy;
        lsm6ds3_gy_flag_data_ready_get(dev_ctx, &drdy);
        if (!drdy) { i--; continue; }

        lsm6ds3_angular_rate_raw_get(dev_ctx, raw);
        sum[0] += raw[0];
        sum[1] += raw[1];
        sum[2] += raw[2];

        platform_delay(2);
    }

    gyro_bias[0] = (float)sum[0] / samples;
    gyro_bias[1] = (float)sum[1] / samples;
    gyro_bias[2] = (float)sum[2] / samples;
}

stmdev_ctx_t dev_ctx = {
        .write_reg = platform_write,
        .read_reg = platform_read,
        .mdelay = platform_delay,
        .handle = &i2cPort
    };

void init_lsm6ds3(void)
{
    platform_init();
    platform_delay(10);

    lsm6ds3_device_id_get(&dev_ctx, &whoamI);
    printf("whoami: %d\n", whoamI);
    // if (whoamI != LSM6DS3_ID)
    //     while(1);

    lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
        lsm6ds3_reset_get(&dev_ctx, &rst);
    } while (rst);

    lsm6ds3_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    lsm6ds3_xl_full_scale_set(&dev_ctx, LSM6DS3_2g);
    lsm6ds3_gy_full_scale_set(&dev_ctx, LSM6DS3_2000dps);

    lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_104Hz);
    lsm6ds3_gy_data_rate_set(&dev_ctx, LSM6DS3_GY_ODR_104Hz);

    calibrate_gyro(&dev_ctx);
}

void poll_lsm6ds3(void)
{
    uint8_t reg;

    /* Accelerometer */
    lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &reg);
    if (reg)
    {
        memset(data_raw_acceleration, 0, sizeof(data_raw_acceleration));
        lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

        acceleration_mg[0] = 9.81f * lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[0]) / 1000;
        acceleration_mg[1] = 9.81f * lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[1]) / 1000;
        acceleration_mg[2] = 9.81f * lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[2]) / 1000;
    }

    /* Gyroscope */
    lsm6ds3_gy_flag_data_ready_get(&dev_ctx, &reg);
    if (reg)
    {
        memset(data_raw_angular_rate, 0, sizeof(data_raw_angular_rate));
        lsm6ds3_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);

        data_raw_angular_rate[0] -= (int16_t)gyro_bias[0];
        data_raw_angular_rate[1] -= (int16_t)gyro_bias[1];
        data_raw_angular_rate[2] -= (int16_t)gyro_bias[2];

        angular_rate_mdps[0] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) / 1000;
        angular_rate_mdps[1] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) / 1000;
        angular_rate_mdps[2] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) / 1000;

        if (!last_time)
            last_time = esp_timer_get_time();

        int64_t time_now = esp_timer_get_time();
        double dt = (double)(time_now - last_time) / 1e6;

        estimate_position_orientation(acceleration_mg, angular_rate_mdps, dt);

        // printf("ACC [m/s^2]: %.3f  %.3f  %.3f | GYRO [dps]: %.3f  %.3f  %.3f | ORI [deg]: roll=%.2f pitch=%.2f yaw=%.2f\n",
        //     acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
        //     angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
        //     orientation[0], orientation[1], orientation[2]);

        last_time = time_now;
    }
}

// LSM6DS3 Polling Loop
void lsm6ds3_read_data_polling(void)
{
    init_lsm6ds3();

    while (1)
    {
        poll_lsm6ds3();
    }
}

#endif
