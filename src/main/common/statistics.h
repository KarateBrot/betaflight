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

// Library for efficient real-time statistics on data streams
// Memory complexity:  O(1)
// Runtime complexity: O(1)

#pragma once

#include <stdint.h>

#include "common/filter.h"

// Moving mean
typedef struct movMean_s {
    pt1Filter_t filter;
} movMean_t;

// Moving root mean square
typedef struct movRms_s {
    float value;
    movMean_t meanXX;
} movRms_t;

// Moving variance
typedef struct movVar_s {
    float value;
    movMean_t meanX;
    movMean_t meanXX;
} movVar_t;

// Moving covariance
typedef struct movCovar_s {
    float value;
    movMean_t meanX;
    movMean_t meanY;
    movMean_t meanXY;
} movCovar_t;

// Moving correlation
typedef struct movCorr_s {
    float value;
    movVar_t varX;
    movVar_t varY;
    movCovar_t covarXY;
} movCorr_t;

void movMeanInit(movMean_t *mean, const float windowSizeUs, const float looprateHz);
void movMeanPush(movMean_t *mean, const float value);
float movMeanGet(const movMean_t *mean);

void movRmsInit(movRms_t *rms, const float windowSizeUs, const float looprateHz);
void movRmsPush(movRms_t *rms, const float value);
float movRmsGet(const movRms_t *rms);

void movVarInit(movVar_t *var, const float windowSizeUs, const float looprateHz);
void movVarPush(movVar_t *var, const float value);
float movVarGet(const movVar_t *var);

void movCovarInit(movCovar_t *covar, const float windowSizeUs, const float looprateHz);
void movCovarPush(movCovar_t *covar, const float valueX, const float valueY);
float movCovarGet(const movCovar_t *covar);

void movCorrInit(movCorr_t *corr, const float windowSizeUs, const float looprateHz);
void movCorrPush(movCorr_t *corr, const float valueX, const float valueY);
float movCorrGet(const movCorr_t *corr);
