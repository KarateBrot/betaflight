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

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LOADCELL_NONE = 0,
    LOADCELL_HX711,
} loadcellHardware_e;

struct loadcellDev_s;

typedef struct loadcellVTable_s {
    bool (*read)(struct loadcellDev_s *dev);
    bool (*tare)(struct loadcellDev_s *dev);
} loadcellVTable_t;

typedef struct loadcellDev_s {
    loadcellVTable_t *vTable;
    loadcellHardware_e hardware;
    int32_t loadADC;
    int32_t offset;
    float scale;
} loadcellDev_t;

