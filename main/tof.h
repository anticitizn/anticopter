#ifndef ANTICOPTER_TOF
#define ANTICOPTER_TOF

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "vl53l1x/VL53L1X_api.h"
#include "vl53l1x/vl53l1_platform.h"


/*
 * ST ULD address format.
 *
 * 0x52 is the 8-bit write address.
 * The platform layer converts it to ESP-IDF's 7-bit 0x29.
 */
#define TOF_DEVICE_ADDRESS          0x52

#define TOF_BOOT_TIMEOUT_MS         1000
#define TOF_TIMING_BUDGET_MS        20
#define TOF_INTERMEASUREMENT_MS     25

#define TOF_DISTANCE_MODE_SHORT     1
#define TOF_DISTANCE_MODE_LONG      2

/*
 * Current ST ULD documentation specifies 0xEACC
 * for VL53L1X_GetSensorId().
 */
#define TOF_EXPECTED_SENSOR_ID      0xEACC


typedef struct
{
    uint16_t distance_mm;
    uint16_t ambient;
    uint16_t signal_per_spad;
    uint16_t spad_count;
    uint8_t range_status;

    bool valid;
    bool new_data;

    int64_t timestamp_us;

} tof_data_t;


static const char *TOF_TAG = "VL53L1X";

static tof_data_t tof_data = {0};
static bool tof_initialized = false;


/* ---------------------------------------------------------
 * Helpers
 * --------------------------------------------------------- */

static esp_err_t tof_check_status(
    VL53L1X_ERROR status,
    const char *operation)
{
    if (status != 0) {
        ESP_LOGE(
            TOF_TAG,
            "%s failed: %d",
            operation,
            (int)status);

        return ESP_FAIL;
    }

    return ESP_OK;
}


/* ---------------------------------------------------------
 * Initialization
 * --------------------------------------------------------- */

esp_err_t tof_init(void)
{
    VL53L1X_ERROR status;
    uint8_t booted = 0;
    uint16_t sensor_id = 0;

    tof_initialized = false;
    tof_data = (tof_data_t){0};


    /*
     * Initialize the ESP-IDF I2C bus.
     *
     * VL53L1_PlatformInit() configures:
     *
     *     I2C_NUM_0
     *     SDA = GPIO 2
     *     SCL = GPIO 1
     *     100 kHz
     *
     * according to the platform implementation.
     */
    status = VL53L1_PlatformInit();

    if (status != 0) {
        ESP_LOGE(
            TOF_TAG,
            "I2C platform initialization failed: %d",
            (int)status);

        return ESP_FAIL;
    }


    /*
     * Wait for the VL53L1X firmware to finish booting.
     *
     * Use a timeout so a missing/disconnected sensor cannot
     * block startup indefinitely.
     */
    int64_t start_us = esp_timer_get_time();

    while (!booted) {

        status = VL53L1X_BootState(
            TOF_DEVICE_ADDRESS,
            &booted);

        if (status != 0) {
            ESP_LOGE(
                TOF_TAG,
                "BootState failed: %d",
                (int)status);

            return ESP_FAIL;
        }

        if ((esp_timer_get_time() - start_us) >
            (TOF_BOOT_TIMEOUT_MS * 1000LL)) {

            ESP_LOGE(
                TOF_TAG,
                "Sensor boot timeout");

            return ESP_ERR_TIMEOUT;
        }

        if (!booted) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }


    /*
     * Verify that the device responding on 0x52 is a VL53L1X.
     */
    status = VL53L1X_GetSensorId(
        TOF_DEVICE_ADDRESS,
        &sensor_id);

    if (tof_check_status(
            status,
            "GetSensorId") != ESP_OK) {

        return ESP_FAIL;
    }


    if (sensor_id != TOF_EXPECTED_SENSOR_ID) {

        ESP_LOGE(
            TOF_TAG,
            "Unexpected sensor ID: 0x%04X "
            "(expected 0x%04X)",
            sensor_id,
            TOF_EXPECTED_SENSOR_ID);

        return ESP_ERR_NOT_FOUND;
    }


    /*
     * Load the VL53L1X default configuration.
     */
    status = VL53L1X_SensorInit(
        TOF_DEVICE_ADDRESS);

    if (tof_check_status(
            status,
            "SensorInit") != ESP_OK) {

        return ESP_FAIL;
    }


    /*
     * Long mode provides maximum ranging distance.
     *
     * Short mode can be preferable for a downward-facing
     * altitude sensor when operating close to the ground or
     * under strong ambient illumination.
     */
    status = VL53L1X_SetDistanceMode(
        TOF_DEVICE_ADDRESS,
        TOF_DISTANCE_MODE_LONG);

    if (tof_check_status(
            status,
            "SetDistanceMode") != ESP_OK) {

        return ESP_FAIL;
    }


    /*
     * Configure the measurement timing budget.
     */
    status = VL53L1X_SetTimingBudgetInMs(
        TOF_DEVICE_ADDRESS,
        TOF_TIMING_BUDGET_MS);

    if (tof_check_status(
            status,
            "SetTimingBudget") != ESP_OK) {

        return ESP_FAIL;
    }


    /*
     * The intermeasurement period must be at least as long
     * as the timing budget.
     */
    status = VL53L1X_SetInterMeasurementInMs(
        TOF_DEVICE_ADDRESS,
        TOF_INTERMEASUREMENT_MS);

    if (tof_check_status(
            status,
            "SetInterMeasurement") != ESP_OK) {

        return ESP_FAIL;
    }


    /*
     * Begin continuous timed ranging.
     */
    status = VL53L1X_StartRanging(
        TOF_DEVICE_ADDRESS);

    if (tof_check_status(
            status,
            "StartRanging") != ESP_OK) {

        return ESP_FAIL;
    }


    tof_initialized = true;

    ESP_LOGI(
        TOF_TAG,
        "Initialized: ID=0x%04X, "
        "budget=%d ms, period=%d ms",
        sensor_id,
        TOF_TIMING_BUDGET_MS,
        TOF_INTERMEASUREMENT_MS);

    return ESP_OK;
}


/* ---------------------------------------------------------
 * Poll sensor
 * --------------------------------------------------------- */

void tof_poll(void)
{
    if (!tof_initialized) {
        return;
    }


    uint8_t data_ready = 0;

    VL53L1X_ERROR status =
        VL53L1X_CheckForDataReady(
            TOF_DEVICE_ADDRESS,
            &data_ready);


    if (status != 0) {

        ESP_LOGW(
            TOF_TAG,
            "CheckForDataReady failed: %d",
            (int)status);

        return;
    }


    if (!data_ready) {
        return;
    }


    VL53L1X_Result_t result = {0};

    status = VL53L1X_GetResult(
        TOF_DEVICE_ADDRESS,
        &result);


    /*
     * Clear the interrupt after consuming a completed
     * ranging cycle.
     *
     * Attempt this even if GetResult() returned an error so
     * the sensor does not remain stuck in its ready state.
     */
    VL53L1X_ERROR clear_status =
        VL53L1X_ClearInterrupt(
            TOF_DEVICE_ADDRESS);


    if (status != 0) {

        ESP_LOGW(
            TOF_TAG,
            "GetResult failed: %d",
            (int)status);

        return;
    }


    if (clear_status != 0) {

        ESP_LOGW(
            TOF_TAG,
            "ClearInterrupt failed: %d",
            (int)clear_status);
    }


    tof_data.distance_mm =
        result.Distance;

    tof_data.ambient =
        result.Ambient;

    tof_data.signal_per_spad =
        result.SigPerSPAD;

    tof_data.spad_count =
        result.NumSPADs;

    tof_data.range_status =
        result.Status;

    tof_data.timestamp_us =
        esp_timer_get_time();


    /*
     * ULD range status 0 is a valid measurement.
     *
     * Invalid measurements should normally not be fed into
     * altitude estimation/control.
     */
    tof_data.valid =
        (result.Status == 0);

    tof_data.new_data = true;
    printf("TOF: %d mm\n", tof_data.distance_mm);
}


/* ---------------------------------------------------------
 * Measurement access
 * --------------------------------------------------------- */

bool tof_get_measurement(
    tof_data_t *measurement)
{
    if ((measurement == NULL) ||
        !tof_data.new_data) {

        return false;
    }

    *measurement = tof_data;

    tof_data.new_data = false;

    return true;
}


uint16_t tof_get_distance_mm(void)
{
    return tof_data.distance_mm;
}


bool tof_distance_valid(void)
{
    return tof_data.valid;
}


/* ---------------------------------------------------------
 * Stop ranging
 * --------------------------------------------------------- */

esp_err_t tof_stop(void)
{
    if (!tof_initialized) {
        return ESP_OK;
    }


    VL53L1X_ERROR status =
        VL53L1X_StopRanging(
            TOF_DEVICE_ADDRESS);


    if (status != 0) {

        ESP_LOGE(
            TOF_TAG,
            "StopRanging failed: %d",
            (int)status);

        return ESP_FAIL;
    }


    tof_initialized = false;

    return ESP_OK;
}

#endif