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

#include <math.h>
#include "platform.h"

#ifdef USE_FEEDFORWARD

#include "build/debug.h"

#include "common/maths.h"

#include "fc/rc.h"

#include "flight/pid.h"

#include "sensors/gyro.h"

#include "feedforward.h"

static FAST_DATA_ZERO_INIT float pidLooptimeS;
static FAST_DATA_ZERO_INIT float rxLooptimeS;
static FAST_DATA_ZERO_INIT float setpointDelta[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT float prevSetpoint[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT float prevVel[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT float ffMaxRateLimit[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT pt2Filter_t velLpf[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT pt3Filter_t accLpf[XYZ_AXIS_COUNT];

void feedforwardInit(const pidProfile_t *pidProfile, const timeUs_t targetLooptimeUs)
{
    pidLooptimeS = targetLooptimeUs * 1e-6f;
    rxLooptimeS = getCurrentRxLooptimeUs() * 1e-6f;

    const float ffMaxRateScale = pidProfile->feedforward_max_rate_limit / 100.0f;
    const float rxNyquistHz = 1.0f / (2.0f * rxLooptimeS);
    const float bandwidthHz = rxNyquistHz * pidGetFeedforwardSmoothFactor() / 100.0f;  // smoothFactor > 50 --> Aliasing!

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        ffMaxRateLimit[i] = applyCurve(i, 1.0f) * ffMaxRateScale;
        pt2FilterInit(&velLpf[i], pt2FilterGain(bandwidthHz, pidLooptimeS));
        pt3FilterInit(&accLpf[i], pt3FilterGain(bandwidthHz / 2.0f, pidLooptimeS));
    }
}

FAST_CODE_NOINLINE float feedforwardApply(int axis)
{
    const float ffTransitionFactor = pidGetFeedforwardTransitionFactor();
    const float setpoint = getSetpointRate(axis);
    
    // calculate setpoint velocity and setpoint acceleration
    // don't divide by deltaTime to save multiplying by deltaTime when calculating setpointDelta
    float vel = (setpoint - prevSetpoint[axis]);
    float acc = (vel - prevVel[axis]);

    // attenuate high frequency noise from derivatives
    vel = pt2FilterApply(&velLpf[axis], vel);
    acc = pt3FilterApply(&accLpf[axis], acc);

    // make feedforward signal react "harder" to quick changes
    acc *= pidGetFeedforwardBoostFactor();

    // predict the next setpoint change
    // assuming constant acceleration: ds = 0.5*acc*dt² + vel*dt
    setpointDelta[axis] = vel + 0.5 * acc;

    // apply feedforward transition
    setpointDelta[axis] *= ffTransitionFactor > 0 ? MIN(1.0f, getRcDeflectionAbs(axis) * ffTransitionFactor) : 1.0f;

    if (axis == gyro.gyroDebugAxis) {
        DEBUG_SET(DEBUG_FEEDFORWARD, 0, lrintf(setpoint));                      // setpoint
        DEBUG_SET(DEBUG_FEEDFORWARD, 1, lrintf(vel * 100.0f));                  // setpoint change from velocity component
        DEBUG_SET(DEBUG_FEEDFORWARD, 2, lrintf(acc * 50.0f));                   // setpoint change from acceleration component with applied boost
        DEBUG_SET(DEBUG_FEEDFORWARD, 3, lrintf(setpointDelta[axis] * 100.0f));  // predicted setpoint change with applied FF transition
    }

    // buffer old values
    prevSetpoint[axis] = setpoint;
    prevVel[axis] = vel;

    return setpointDelta[axis]; // predicted setpoint change, used by the PID code
}

FAST_CODE_NOINLINE float applyFeedforwardLimit(int axis, float value, float Kp, float currentPidSetpoint)
{
    switch (axis) {
    case FD_ROLL:
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 0, value);
        break;
    case FD_PITCH:
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 1, value);
        break;
    }

    if (value * currentPidSetpoint > 0.0f) {
        if (fabsf(currentPidSetpoint) <= ffMaxRateLimit[axis]) {
            value = constrainf(value, (-ffMaxRateLimit[axis] - currentPidSetpoint) * Kp, (ffMaxRateLimit[axis] - currentPidSetpoint) * Kp);
        } else {
            value = 0;
        }
    }

    if (axis == gyro.gyroDebugAxis) {
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 2, value);
    }

    return value;
}

bool shouldApplyFeedforwardLimits(int axis)
{
    return axis < FD_YAW && ffMaxRateLimit[axis] != 0.0f;
}

#endif // USE_FEEDFORWARD
