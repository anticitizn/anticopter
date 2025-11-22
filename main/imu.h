#ifndef ANTICOPTER_IMU
#define ANTICOPTER_IMU

#include "driver/i2c.h"
#include "esp_log.h"
#include "lsm6ds3/lsm6ds3_reg.h"
#include "lis3mdl/lis3mdl_reg.h"
#include "camera.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/* ---------------------------------------------------------
   I2C Configuration
--------------------------------------------------------- */
#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_SDA_IO 2
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define LSM6DS3_SENSOR_ADDR 0x6A // IMU
#define LIS3MDL_SENSOR_ADDR 0x1C // Magnetometer

/* ---------------------------------------------------------
   Orientation / Filter Configuration
--------------------------------------------------------- */
#define RAD_TO_DEG (180.0f / M_PI)
#define Kp_BASE  2.0f
#define Kp_BOOST 10.0f
#define Ki       0.005f

static float Kp_current = Kp_BASE;
static float ahrs_time  = 0.0f;

/* ---------------------------------------------------------
   Quaternion + Integral Feedback
--------------------------------------------------------- */
static float q[4]        = {1.0f, 0.0f, 0.0f, 0.0f};
static float integralFB[3] = {0.0f, 0.0f, 0.0f};

/* ---------------------------------------------------------
   Raw IMU Data
--------------------------------------------------------- */
static int16_t data_raw_acceleration[3] = {0};
static int16_t data_raw_angular_rate[3] = {0};
static int16_t data_raw_temperature     = 0;

static float acceleration_mg[3] = {0};
static float angular_rate_mdps[3] = {0};
static float temperature_degC = 0.0f;

static float gyro_bias[3] = {0};
static float orientation_offset[3] = {0};
static float orientation[3] = {0};

bool imu_data_ready = false;
static int64_t last_time_imu = 0;

/* ---------------------------------------------------------
   Magnetometer Data
--------------------------------------------------------- */
static int16_t data_raw_magnetic[3] = {0};
static float magnetic_mG[3] = {0};
float mag_norm[3] = {0};
static float mag_temperature_degC = 0;
static bool mag_data_ready = false;

float mag_bias[3]  = {-4639.000000, 1327.500000, -513.500000};
float mag_scale[3] = {1.041470, 1.002780, 0.959149};

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

/*--------------------------------------------------------------------
   Mahony Filter
 --------------------------------------------------------------------*/
void MahonyAHRSupdate(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float mx, float my, float mz,
                      float dt)
{
    // Tunable gains
    const float Kp_acc = Kp_current;  // accel correction uses dynamic Kp
    const float Kp_mag = 0.35f;       // smaller constant mag correction gain
    const float Ki_acc = Ki;          // integral only from accel error
    const float alpha_mag = 0.2f;     // magnetometer low-pass alpha (0 < alpha < 1, lower = more smoothing)

    // Persistent magnetometer smoothing
    static float mag_lp[3] = {0.0f, 0.0f, 0.0f};
    static bool  mag_lp_init = false;

    float norm;
    float vx, vy, vz;
    float hx, hy, hz;
    float bx, bz;
    float wx, wy, wz;
    float ex_acc, ey_acc, ez_acc;
    float ex_mag = 0.0f, ey_mag = 0.0f, ez_mag = 0.0f;
    float ex, ey, ez;
    int useMag = 1;

    // --- Normalize accelerometer ---
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) 
    {
        ax /= norm;
        ay /= norm;
        az /= norm;
    } 
    else 
    {
        // invalid accel; ignore its contribution
        ax = ay = az = 0.0f;
    }

    // --- Normalize + low-pass magnetometer ---
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm > 0.0f) 
    {
        mx /= norm;
        my /= norm;
        mz /= norm;

        if (!mag_lp_init) 
        {
            // Initialize LP filter to first valid sample to avoid a big transient
            mag_lp[0] = mx;
            mag_lp[1] = my;
            mag_lp[2] = mz;
            mag_lp_init = true;
        } 
        else 
        {
            mag_lp[0] = alpha_mag * mx + (1.0f - alpha_mag) * mag_lp[0];
            mag_lp[1] = alpha_mag * my + (1.0f - alpha_mag) * mag_lp[1];
            mag_lp[2] = alpha_mag * mz + (1.0f - alpha_mag) * mag_lp[2];
        }

        // Re-normalize filtered mag
        norm = sqrtf(mag_lp[0] * mag_lp[0] +
                     mag_lp[1] * mag_lp[1] +
                     mag_lp[2] * mag_lp[2]);
        if (norm > 0.0f) 
        {
            mx = mag_lp[0] / norm;
            my = mag_lp[1] / norm;
            mz = mag_lp[2] / norm;
        } 
        else 
        {
            useMag = 0;
        }
    } 
    else 
    {
        useMag = 0; // fall back to 6 DOF if mag is bad
    }

    // Short names for readability
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    // --- Estimated direction of gravity (from quaternion) ---
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // --- Accelerometer error (gravity) ---
    ex_acc = (ay * vz - az * vy);
    ey_acc = (az * vx - ax * vz);
    ez_acc = (ax * vy - ay * vx);

    // --- Magnetometer error (heading), horizontal only ---
    if (useMag) 
    {
        // Rotate mag into Earth frame (h = q * m_body * q_conj)
        hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3)
           + 2.0f * my * (q1 * q2 - q0 * q3)
           + 2.0f * mz * (q1 * q3 + q0 * q2);

        hy = 2.0f * mx * (q1 * q2 + q0 * q3)
           + 2.0f * my * (0.5f - q1 * q1 - q3 * q3)
           + 2.0f * mz * (q2 * q3 - q0 * q1);

        hz = 2.0f * mx * (q1 * q3 - q0 * q2)
           + 2.0f * my * (q2 * q3 + q0 * q1)
           + 2.0f * mz * (0.5f - q1 * q1 - q2 * q2);

        // Only use horizontal field for heading (zero vertical component)
        hx = hx;
        hy = hy;
        hz = 0.0f;

        // Horizontal magnitude
        bx = sqrtf(hx * hx + hy * hy);
        if (bx < 1e-6f) 
        {
            // Degenerate case; skip mag correction
            useMag = 0;
        } 
        else 
        {
            // Expected magnetic field direction (body frame) using horizontal-only reference
            // bz = 0.0f effectively decouples pitch/roll from mag
            bz = 0.0f;

            wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3)
               + 2.0f * bz * (q1 * q3 - q0 * q2);
            wy = 2.0f * bx * (q1 * q2 - q0 * q3)
               + 2.0f * bz * (q0 * q1 + q2 * q3);
            wz = 2.0f * bx * (q0 * q2 + q1 * q3)
               + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2);

            // Normalize expected field
            norm = sqrtf(wx * wx + wy * wy + wz * wz);
            if (norm > 0.0f) {
                wx /= norm;
                wy /= norm;
                wz /= norm;
            }

            // Magnetometer error is cross product between measured and expected field
            ex_mag = (my * wz - mz * wy);
            ey_mag = (mz * wx - mx * wz);
            ez_mag = (mx * wy - my * wx);
        }
    }

    // --- Total error: accel + mag ---
    ex = ex_acc + ex_mag;
    ey = ey_acc + ey_mag;
    ez = ez_acc + ez_mag;

    // --- Integral feedback (uses accel only, to avoid slow mag drift) ---
    if (Ki_acc > 0.0f) 
    {
        integralFB[0] += Ki_acc * ex_acc * dt;
        integralFB[1] += Ki_acc * ey_acc * dt;
        integralFB[2] += Ki_acc * ez_acc * dt;

        gx += integralFB[0];
        gy += integralFB[1];
        gz += integralFB[2];
    }

    // --- Proportional feedback: accel + weakened mag ---
    gx += Kp_acc * ex_acc + Kp_mag * ex_mag;
    gy += Kp_acc * ey_acc + Kp_mag * ey_mag;
    gz += Kp_acc * ez_acc + Kp_mag * ez_mag;

    // --- Integrate quaternion rate ---
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qDot0 = -q1 * gx - q2 * gy - q3 * gz;
    float qDot1 =  q0 * gx + q2 * gz - q3 * gy;
    float qDot2 =  q0 * gy - q1 * gz + q3 * gx;
    float qDot3 =  q0 * gz + q1 * gy - q2 * gx;

    q0 += qDot0;
    q1 += qDot1;
    q2 += qDot2;
    q3 += qDot3;

    // --- Normalize quaternion ---
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0f) 
    {
        norm = 1.0f / norm;
        q[0] = q0 * norm;
        q[1] = q1 * norm;
        q[2] = q2 * norm;
        q[3] = q3 * norm;
    } else 
    {
        // Fallback: keep the old quaternion values if normalization fails
        // (this should basically never happen)
    }
}



static void quat_to_euler_deg(const float q[4], float euler_deg[3])
{
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    float roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    float yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                         1.0f - 2.0f * (q2 * q2 + q3 * q3));

    euler_deg[0] = roll  * RAD_TO_DEG;
    euler_deg[1] = pitch * RAD_TO_DEG;
    euler_deg[2] = yaw   * RAD_TO_DEG;
}

void estimate_position_orientation(float accel[3], float gyro[3], float mag[3], double dt)
{
    // Early-time Kp boost with linear ramp-down
    ahrs_time += (float)dt;

    if (ahrs_time < 0.5f) 
    {
        Kp_current = Kp_BOOST;
    } 
    else if (ahrs_time < 1.0f) 
    {
        float t = (ahrs_time - 0.5f) / 0.5f;  // 0 .. 1
        Kp_current = Kp_BOOST + t * (Kp_BASE - Kp_BOOST);
    } 
    else 
    {
        Kp_current = Kp_BASE;
    }

    float gyro_rad[3];
    gyro_rad[0] = gyro[0] * (float)M_PI / 180.0f;
    gyro_rad[1] = gyro[1] * (float)M_PI / 180.0f;
    gyro_rad[2] = gyro[2] * (float)M_PI / 180.0f;

    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    if (mag_data_ready) 
    {
        mx = mag[0];
        my = mag[1];
        mz = mag[2];
    }

    MahonyAHRSupdate(
        gyro_rad[0], gyro_rad[1], gyro_rad[2],
        accel[0],    accel[1],    accel[2],
        mx,          my,          mz,
        (float)dt
    );

    float euler_deg[3];
    quat_to_euler_deg(q, euler_deg);

    orientation[0] = euler_deg[0] - orientation_offset[0];
    orientation[1] = euler_deg[1] - orientation_offset[1];
    orientation[2] = euler_deg[2] - orientation_offset[2];
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

    lsm6ds3_xl_data_rate_set(&dev_ctx_imu, LSM6DS3_XL_ODR_104Hz);
    lsm6ds3_gy_data_rate_set(&dev_ctx_imu, LSM6DS3_GY_ODR_104Hz);

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

    lis3mdl_data_rate_set(&dev_ctx_mag, LIS3MDL_UHP_155Hz);
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

        acceleration_mg[0] = 9.81f * lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[0]) / 1000.0f;
        acceleration_mg[1] = 9.81f * lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[1]) / 1000.0f;
        acceleration_mg[2] = 9.81f * lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[2]) / 1000.0f;
        imu_data_ready = true;
    }

    lsm6ds3_gy_flag_data_ready_get(&dev_ctx_imu, &reg);
    if (reg)
    {
        lsm6ds3_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate);

        data_raw_angular_rate[0] -= gyro_bias[0];
        data_raw_angular_rate[1] -= gyro_bias[1];
        data_raw_angular_rate[2] -= gyro_bias[2];

        angular_rate_mdps[0] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) / 1000.0f;
        angular_rate_mdps[1] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) / 1000.0f;
        angular_rate_mdps[2] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0f;
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

        magnetic_mG[0] = 1000 * lis3mdl_from_fs16_to_gauss(data_raw_magnetic[0]);
        magnetic_mG[1] = 1000 * lis3mdl_from_fs16_to_gauss(data_raw_magnetic[1]);
        magnetic_mG[2] = 1000 * lis3mdl_from_fs16_to_gauss(data_raw_magnetic[2]);

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
void imu_poll(void)
{
    poll_lsm6ds3();
    poll_lis3mdl();

    if (!last_time_imu)
    {
        last_time_imu = esp_timer_get_time();
    }

    int64_t now = esp_timer_get_time();
    double dt = (double)(now - last_time_imu) / 1e6;

    last_time_imu = now;

    if (imu_data_ready && mag_data_ready)
    {
        estimate_position_orientation(acceleration_mg, angular_rate_mdps, mag_norm, dt);
    }
}

// Initialize LSM6DS3 accelerometer + gyro and LIS3MDL magnetometer
void imu_init(void)
{
    init_lsm6ds3();
    init_lis3mdl();

    // Reset AHRS internal state
    ahrs_time = 0.0f;
    q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
    integralFB[0] = integralFB[1] = integralFB[2] = 0.0f;
    last_time_imu = 0;

    // Give the filter some time to converge in the current pose
    for (int i = 0; i < 100; ++i)
    {
        imu_poll();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    float euler_deg[3];
    quat_to_euler_deg(q, euler_deg);

    // Define current orientation as zero
    orientation_offset[0] = euler_deg[0];
    orientation_offset[1] = euler_deg[1];
    orientation_offset[2] = euler_deg[2];
}

#endif
