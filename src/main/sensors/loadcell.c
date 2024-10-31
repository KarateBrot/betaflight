/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_LOADCELL

#include "build/debug.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/loadcell/loadcell_hx711.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"

#include "loadcell.h"

typedef enum {
    LOADCELL_STATE_ERROR = 0,
    LOADCELL_STATE_DETECTED,
    LOADCELL_STATE_COUNT
} loadcellState_e;

FAST_DATA_ZERO_INIT loadcell_t loadcell;

static bool isReady = false;
static loadcellState_e state = LOADCELL_STATE_ERROR;


static bool loadcellDetect(loadcellDev_t *loadDev, const loadcellHardware_e hardware)
{
    loadcellHardware_e hardwareDetected = LOADCELL_NONE;

    switch (hardware) {

        case LOADCELL_HX711:
#ifdef USE_LOADCELL_HX711
            if (loadcellHx711Detect(loadDev)) {
                hardwareDetected = LOADCELL_HX711;
                break;
            }
#endif // USE_LOADCELL_HX711
            FALLTHROUGH;

        case LOADCELL_NONE:
            hardwareDetected = LOADCELL_NONE;
            break;
    }

    if (hardwareDetected == LOADCELL_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_LOADCELL] = hardwareDetected;
    sensorsSet(SENSOR_LOADCELL);

    return true;
}


void loadcellInit(const loadcellConfig_t *config)
{   
    isReady = loadcellDetect(&loadcell.dev, config->loadcell_hardware);

    if (isReady) {
        state = LOADCELL_STATE_DETECTED;
    }

    DEBUG_SET(DEBUG_LOADCELL, 0, isReady);
}


void loadcellUpdate(void)
{
    static int16_t n = 0;
    DEBUG_SET(DEBUG_LOADCELL, 1, n++);

    switch (state) {
        case LOADCELL_STATE_DETECTED:
            break;
        case LOADCELL_STATE_ERROR:
        default:
            break;
    }
}


void loadcellTare(void)
{
    return;
}

bool loadcellIsReady(void)
{
    return isReady;
}

#endif // USE_LOADCELL
