#ifndef ANTICOPTER_IMU
#define ANTICOPTER_IMU

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "lsm6ds3/lsm6ds3_reg.h"
#include "lis3mdl/lis3mdl_reg.h"
#include "camera.h"
#include "comms/msg_send.h"
#include "Fusion/Fusion.h"

/* ---------------------------------------------------------
   I2C Configuration
--------------------------------------------------------- */
#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_SDA_IO 2
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define LSM6DS3_SENSOR_ADDR 0x6A // IMU
#define LIS3MDL_SENSOR_ADDR 0x1C // Magnetometer

// Raw IMU data storage
static int16_t data_raw_acceleration[3] = {0};
static int16_t data_raw_angular_rate[3] = {0};
static int16_t data_raw_temperature     = 0;

static float acceleration_g[3] = {0};
static float angular_rate_dps[3] = {0};
static float temperature_degC = 0.0f;

static float gyro_bias[3] = {0};
static float orientation_offset[3] = {0};
static float orientation[3] = {0};

bool imu_data_ready = false;
static int64_t last_time_imu = 0;

// This is the IMU to drone body rotation matrix
// currently just identity because the values that I got out of it with least squares
// were worse than the identity matrix...
static const float R_mount_matrix[3][3] = {
    {  1,  0,  0 },
    {  0,  1,  0 },
    {  0,  0,  1 }
};

// Magnetometer data
static int16_t data_raw_magnetic[3] = {0};
static float magnetic_mG[3] = {0};
float mag_norm[3] = {0};
static float mag_temperature_degC = 0;
static bool mag_data_ready = false;

float mag_bias[3]  = {-4639.000000, 1327.500000, -513.500000};
float mag_scale[3] = {1.041470, 1.002780, 0.959149};

// Attitude estimation stuff
FusionAhrs ahrs;
FusionAhrsSettings ahrsSettings;

/* ---------------------------------------------------------
   Contexts and Buffers
--------------------------------------------------------- */
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

static i2c_port_t i2cPort = I2C_MASTER_NUM;

static stmdev_ctx_t dev_ctx_imu;
static stmdev_ctx_t dev_ctx_mag;

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
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE,
        0);
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    uint8_t addr = (uint32_t)handle;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, (uint8_t *)bufp, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    uint8_t addr = (uint32_t)handle;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
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
    return; // stay silent
}

static inline void apply_mount_matrix(float v[3], const float R[3][3])
{
    float x = v[0];
    float y = v[1];
    float z = v[2];

    v[0] = R[0][0]*x + R[0][1]*y + R[0][2]*z;
    v[1] = R[1][0]*x + R[1][1]*y + R[1][2]*z;
    v[2] = R[2][0]*x + R[2][1]*y + R[2][2]*z;
}


static void calibrate_gyro(stmdev_ctx_t *dev_ctx)
{
    const int samples = 500;
    int32_t sum[3] = {0, 0, 0};
    int16_t raw[3];

    // let the sensors settle
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

void init_lsm6ds3(void)
{
    platform_init();
    platform_delay(10);

    dev_ctx_imu.write_reg = platform_write;
    dev_ctx_imu.read_reg  = platform_read;
    dev_ctx_imu.mdelay    = platform_delay;
    dev_ctx_imu.handle    = (void*)LSM6DS3_SENSOR_ADDR;

    lsm6ds3_device_id_get(&dev_ctx_imu, &whoamI);

    lsm6ds3_reset_set(&dev_ctx_imu, PROPERTY_ENABLE);
    do {
        lsm6ds3_reset_get(&dev_ctx_imu, &rst);
    } while (rst);

    lsm6ds3_block_data_update_set(&dev_ctx_imu, PROPERTY_ENABLE);

    lsm6ds3_xl_full_scale_set(&dev_ctx_imu, LSM6DS3_2g);
    lsm6ds3_gy_full_scale_set(&dev_ctx_imu, LSM6DS3_2000dps);

    lsm6ds3_xl_data_rate_set(&dev_ctx_imu, LSM6DS3_XL_ODR_416Hz);
    lsm6ds3_gy_data_rate_set(&dev_ctx_imu, LSM6DS3_GY_ODR_416Hz);

    calibrate_gyro(&dev_ctx_imu);
}

void init_lis3mdl(void)
{
    dev_ctx_mag.write_reg = platform_write;
    dev_ctx_mag.read_reg  = platform_read;
    dev_ctx_mag.mdelay    = platform_delay;
    dev_ctx_mag.handle    = (void*)LIS3MDL_SENSOR_ADDR;

    lis3mdl_device_id_get(&dev_ctx_mag, &whoamI);

    lis3mdl_reset_set(&dev_ctx_mag, PROPERTY_ENABLE);
    do {
        lis3mdl_reset_get(&dev_ctx_mag, &rst);
    } while (rst);

    lis3mdl_block_data_update_set(&dev_ctx_mag, PROPERTY_ENABLE);

    lis3mdl_data_rate_set(&dev_ctx_mag, LIS3MDL_MP_560Hz);
    lis3mdl_full_scale_set(&dev_ctx_mag, LIS3MDL_16_GAUSS);
    lis3mdl_temperature_meas_set(&dev_ctx_mag, PROPERTY_ENABLE);
    lis3mdl_operating_mode_set(&dev_ctx_mag, LIS3MDL_CONTINUOUS_MODE);
}

void poll_lsm6ds3(void)
{
    uint8_t reg;

    lsm6ds3_xl_flag_data_ready_get(&dev_ctx_imu, &reg);
    if (reg)
    {
        lsm6ds3_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration);

        // printf("%d %d %d\n", data_raw_acceleration[0], data_raw_acceleration[1], data_raw_acceleration[2]);

        acceleration_g[0] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[0]) / 1000.0f;
        acceleration_g[1] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[1]) / 1000.0f;
        acceleration_g[2] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[2]) / 1000.0f;

        apply_mount_matrix(acceleration_g, R_mount_matrix);

        imu_data_ready = true;
    }

    lsm6ds3_gy_flag_data_ready_get(&dev_ctx_imu, &reg);
    if (reg)
    {
        lsm6ds3_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate);

        data_raw_angular_rate[0] -= gyro_bias[0];
        data_raw_angular_rate[1] -= gyro_bias[1];
        data_raw_angular_rate[2] -= gyro_bias[2];

        angular_rate_dps[0] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) / 1000.0f;
        angular_rate_dps[1] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) / 1000.0f;
        angular_rate_dps[2] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0f;

        apply_mount_matrix(angular_rate_dps, R_mount_matrix);

        imu_data_ready = true;
    }
}

void poll_lis3mdl(void)
{
    uint8_t reg;
    lis3mdl_mag_data_ready_get(&dev_ctx_mag, &reg);

    if (reg)
    {
        lis3mdl_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic);

        data_raw_magnetic[0] -= mag_bias[0];
        data_raw_magnetic[1] -= mag_bias[1];
        data_raw_magnetic[2] -= mag_bias[2];

        data_raw_magnetic[0] *= mag_scale[0];
        data_raw_magnetic[1] *= mag_scale[1];
        data_raw_magnetic[2] *= mag_scale[2];

        //printf("Mag raw: %d %d %d\n", data_raw_magnetic[0], data_raw_magnetic[1], data_raw_magnetic[2]);

        magnetic_mG[0] = 1000 * lis3mdl_from_fs16_to_gauss(data_raw_magnetic[0]);
        magnetic_mG[1] = 1000 * lis3mdl_from_fs16_to_gauss(data_raw_magnetic[1]);
        magnetic_mG[2] = 1000 * lis3mdl_from_fs16_to_gauss(data_raw_magnetic[2]);

        apply_mount_matrix(magnetic_mG, R_mount_matrix);

        lis3mdl_temperature_raw_get(&dev_ctx_mag, &data_raw_temperature);
        mag_temperature_degC = lis3mdl_from_lsb_to_celsius(data_raw_temperature);

        float norm = sqrtf(
            magnetic_mG[0]*magnetic_mG[0] +
            magnetic_mG[1]*magnetic_mG[1] +
            magnetic_mG[2]*magnetic_mG[2]
        );
        
        // printf("%d, %d, %d\n", data_raw_magnetic[0], data_raw_magnetic[1], data_raw_magnetic[2]);

        if (norm > 0.0f)
        {
            mag_norm[0] = magnetic_mG[0] / norm;
            mag_norm[1] = magnetic_mG[1] / norm;
            mag_norm[2] = magnetic_mG[2] / norm;

            mag_data_ready = true;
        }
        else
        {
            mag_data_ready = false;
        }

    }
}

// Poll LSM6DS3 accelerometer + gyro and LIS3MDL magnetometer
bool imu_poll(void)
{
    poll_lsm6ds3();
    poll_lis3mdl();

    if (imu_data_ready)
    {
        int64_t now = esp_timer_get_time();
        double dt = (double)(now - last_time_imu) / 1e6;

        last_time_imu = now;

        const FusionVector gyroscope = {.array = {angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2]}};
        const FusionVector accelerometer = {.array = {acceleration_g[0], acceleration_g[1], acceleration_g[2]}};

        FusionAhrsSetSamplePeriod(&ahrs, (float)dt);
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        orientation[0] = euler.angle.roll;
        orientation[1] = euler.angle.pitch;
        orientation[2] = euler.angle.yaw;

        imu_data_ready = false;
        mag_data_ready = false;

        printf("Dt: %lf\n", dt);
        
        return true;
    }

    return false;
}

void init_ahrs(void)
{
    FusionAhrsInitialise(&ahrs);

    ahrsSettings = fusionAhrsDefaultSettings;
    ahrsSettings.sampleRate = 400.0f; // Hz
    FusionAhrsSetSettings(&ahrs, &ahrsSettings);
}

// Initialize LSM6DS3 accelerometer + gyro and LIS3MDL magnetometer
void imu_init(void)
{
    init_lsm6ds3();
    init_lis3mdl();

    init_ahrs();

    last_time_imu = esp_timer_get_time();
}

void handle_imu_telemetry(const void *payload)
{
    msg_header_t msg_header = {
        .msg_type = MSG_IMU,
        .payload_len = sizeof(msg_imu_t),
    };

    msg_imu_t msg_imu = {0};

    memcpy(msg_imu.acceleration_g, acceleration_g, sizeof(msg_imu.acceleration_g));
    memcpy(msg_imu.angular_rate_dps, angular_rate_dps, sizeof(msg_imu.angular_rate_dps));
    memcpy(msg_imu.orientation, orientation, sizeof(msg_imu.orientation));
    memcpy(msg_imu.magnetic_mG, magnetic_mG, sizeof(msg_imu.magnetic_mG));
    memcpy(msg_imu.mag_norm, mag_norm, sizeof(msg_imu.mag_norm));
    msg_imu.temperature_degC = temperature_degC;
    
    // ESP_LOGI("IMU", "Sending acceleration: %f, %f, %f", msg_imu.acceleration_g[0], msg_imu.acceleration_g[1], msg_imu.acceleration_g[2]);
    send_message(msg_header, &msg_imu);
    
}

void handle_cfg_imu(const void *payload)
{
    msg_cfg_imu_t* msg = (msg_cfg_imu_t*)payload;

    memcpy(&mag_bias, &msg->mag_bias, sizeof(mag_bias));
    memcpy(&mag_scale, &msg->mag_scale, sizeof(mag_scale));
}

#endif
