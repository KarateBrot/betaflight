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

#include <math.h>

#include "platform.h"

#include "common/maths.h"

#include "statistics.h"


void movMeanInit(movMean_t *mean, const float windowSizeUs, const float looprateHz)
{
    const float n = windowSizeUs * looprateHz * 1e-6f;

    // Similar response to a moving average over n samples
    pt1FilterInit(&mean->filter, 2.0f / (MAX(1.0f, n) + 1));
}

void movMeanPush(movMean_t *mean, const float value)
{
    pt1FilterApply(&mean->filter, value);
}

float movMeanGet(const movMean_t *mean)
{
    return mean->filter.state;
}


void movRmsInit(movRms_t *rms, const float windowSizeUs, const float looprateHz)
{
    rms->value = 0.0f;
    movMeanInit(&rms->meanXX, windowSizeUs, looprateHz);
}

void movRmsPush(movRms_t *rms, const float value)
{
    movMeanPush(&rms->meanXX, sq(value));

    if (movMeanGet(&rms->meanXX) <= 0.0f) {
        rms->value = 0.0f;
        return;
    }

    // Rms(X) = sqrt(E(X²))
    rms->value = sqrt_approx(movMeanGet(&rms->meanXX));
}

float movRmsGet(const movRms_t *rms)
{
    return rms->value;
}


void movVarInit(movVar_t *var, const float windowSizeUs, const float looprateHz)
{
    var->value = 0.0f;

    movMeanInit(&var->meanX, windowSizeUs, looprateHz);
    movMeanInit(&var->meanXX, windowSizeUs, looprateHz);
}

void movVarPush(movVar_t *var, const float value)
{
    movMeanPush(&var->meanX, value);
    movMeanPush(&var->meanXX, sq(value));

    // Var(X) = Cov(X,X) = E(X²) - E²(X)
    var->value = movMeanGet(&var->meanXX) - sq(movMeanGet(&var->meanX));

    if (var->value < 0.0f) {
        var->value = 0.0f;
    }
}

float movVarGet(const movVar_t *var)
{
    return var->value;
}


void movCovarInit(movCovar_t *covar, const float windowSizeUs, const float looprateHz)
{
    covar->value = 0.0f;

    movMeanInit(&covar->meanX, windowSizeUs, looprateHz);
    movMeanInit(&covar->meanY, windowSizeUs, looprateHz);
    movMeanInit(&covar->meanXY, windowSizeUs, looprateHz);
}

void movCovarPush(movCovar_t *covar, const float valueX, const float valueY)
{
    movMeanPush(&covar->meanX, valueX);
    movMeanPush(&covar->meanY, valueY);
    movMeanPush(&covar->meanXY, valueX * valueY);

    // Cov(X,Y) = E(XY) - E(X)E(Y)
    covar->value = movMeanGet(&covar->meanXY) - movMeanGet(&covar->meanX) * movMeanGet(&covar->meanY);
}

float movCovarGet(const movCovar_t *covar)
{
    return covar->value;
}


void movCorrInit(movCorr_t *corr, const float windowSizeUs, const float looprateHz)
{
    corr->value = 0.0f;

    movVarInit(&corr->varX, windowSizeUs, looprateHz);
    movVarInit(&corr->varY, windowSizeUs, looprateHz);
    movCovarInit(&corr->covarXY, windowSizeUs, looprateHz);
}

void movCorrPush(movCorr_t *corr, const float valueX, const float valueY)
{
    movVarPush(&corr->varX, valueX);
    movVarPush(&corr->varY, valueY);
    movCovarPush(&corr->covarXY, valueX, valueY);

    const float varXvarY = movVarGet(&corr->varX) * movVarGet(&corr->varY);

    if (varXvarY <= 0.0f) {
        return;
    }

    // Corr(X,Y) = Cov(X,Y) / sqrt(Var(X) * Var(Y))
    corr->value = movCovarGet(&corr->covarXY) * invSqrt_approx(varXvarY);
}

float movCorrGet(const movCorr_t *corr)
{
    return corr->value;
}
