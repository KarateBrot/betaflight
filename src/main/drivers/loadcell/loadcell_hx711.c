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

#include "loadcell_hx711.h"

#ifdef USE_LOADCELL_HX711

static bool loadcellHx711Read(loadcellDev_t *loadcell)
{
    UNUSED(dev);
    return false;
}

static bool loadcellHx711Tare(loadcellDev_t *loadcell)
{
    UNUSED(dev);
    return false;
}

bool loadcellHx711Detect(loadcellDev_t *loadcell)
{
    loadcell->vTable->read = loadcellHx711Read;
    loadcell->vTable->tare = loadcellHx711Tare;

    loadcell->hardware = LOADCELL_HX711;

    return true;
}

#endif // USE_LOADCELL_HX711