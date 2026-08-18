
#ifndef ANTICOPTER_PMW3901_H
#define ANTICOPTER_PMW3901_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"


typedef struct motionBurst_s
{
    union
    {
        uint8_t motion;

        struct
        {
            uint8_t frameFrom0     : 1;
            uint8_t runMode        : 2;
            uint8_t reserved1      : 1;
            uint8_t rawFrom0       : 1;
            uint8_t reserved2      : 2;
            uint8_t motionOccurred : 1;
        };
    };

    uint8_t observation;

    int16_t deltaX;
    int16_t deltaY;

    uint8_t squal;

    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;

    uint16_t shutter;

} __attribute__((packed)) motionBurst_t;


/**
 * Initialize the SPI bus and PMW3901.
 *
 * GPIO:
 *   CS   = GPIO0
 *   MISO = GPIO38
 *   MOSI = GPIO44
 *   CLK  = GPIO43
 *
 * @return ESP_OK if initialization succeeds.
 */
esp_err_t pmw3901_init(void);


/**
 * Read the latest accumulated motion burst.
 *
 * @param motion Destination structure.
 *
 * @return ESP_OK on success.
 */
esp_err_t pmw3901_read_motion(motionBurst_t *motion);


/**
 * Returns true after successful initialization.
 */
bool pmw3901_is_initialized(void);

motionBurst_t pmw3901_poll();

void pmw3901_debug_motion_registers(void);


#endif /* ANTICOPTER_PMW3901_H */
