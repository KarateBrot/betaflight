/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_LOADCELL

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/bus_i2c.h"
#include "drivers/loadcell/loadcell.h"

#include "loadcell.h"

#ifndef LOADCELL_I2C_ADDRESS
#define LOADCELL_I2C_ADDRESS 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(loadcellConfig_t, loadcellConfig, PG_LOADCELL_CONFIG, 0);

PG_RESET_TEMPLATE(loadcellConfig_t, loadcellConfig,
    .loadcell_hardware = LOADCELL_NONE,
    .loadcell_i2c_device = I2C_DEV_TO_CFG(LOADCELL_I2C_INSTANCE),
    .loadcell_i2c_address = LOADCELL_I2C_ADDRESS,
    .loadcell_offset = 0,
    .loadcell_scale = 0,
);

#endif // USE_LOADCELL
