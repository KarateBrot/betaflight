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

/* original work by Rav
 * 2018_07 updated by ctzsnooze to post filter, wider Q, different peak detection
 * coding assistance and advice from DieHertz, Rav, eTracer
 * test pilots icr4sh, UAV Tech, Flint723
 * 2020_06 updated by Jan Post to track and filter multiple peaks per axis
 */

#include "platform.h"

#define USE_GYRO_DATA_ANALYSE

#ifdef USE_GYRO_DATA_ANALYSE
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

#include "fc/core.h"

#include "gyroanalyse.h"

// FFT_WINDOW_SIZE defaults to 32 (gyroanalyse.h)
// We get 16 frequency bins from 32 consecutive data values
// Bin 0 is DC and can't be used.
// Only bins 1 to 15 are usable.

// A gyro sample is collected every gyro loop
// maxSampleCount recent gyro values are accumulated and averaged
// to ensure that 32 samples are collected at the right rate for the required FFT bandwidth

// For an 8k gyro loop, at default 600hz max, 6 sequential gyro data points are averaged, FFT runs 1333Hz.
// Upper limit of FFT is half that frequency, eg 666Hz by default.
// At 8k, if user sets a max of 300Hz, int(8000/600) = 13, fftSamplingRateHz = 615Hz, range 307Hz
// Note that lower max requires more samples to be averaged, increasing precision but taking longer to get enough samples.
// For Bosch at 3200Hz gyro, max of 600, int(3200/1200) = 2, fftSamplingRateHz = 1600, range to 800hz
// For Bosch on XClass, better to set a max of 300, int(3200/600) = 5, fftSamplingRateHz = 640, range to 320Hz
//
// When sampleCount reaches maxSampleCount, the averaged gyro value is put into the circular buffer of 32 samples
// At 8k, with 600Hz max, maxSampleCount = 6, this happens every 6 * 0.125us, or every 0.75ms
// Hence to completely replace all 32 samples of the FFT input buffer with clean new data takes 24ms

// The FFT code is split into steps.  It takes 4 gyro loops to calculate the FFT for one axis
//   (gyroDataAnalyseUpdate has 8 steps, but only four breaks)
// Since there are three axes, it takes 12 gyro loops to completely update all axes.
// At 8k, any one axis gets updated at 8000 / 12 or 666hz or every 1.5ms
// In this time, 2 points in the FFT buffer will have changed.
// At 4k, it takes twice as long to update an axis, i.e. each axis updates only every 3ms
// Four points in the buffer will have changed in that time, and each point will be the average of three samples.
// Hence output jitter at 4k is about four times worse than at 8k.  At 2k output jitter is quite bad.

// The window step loads gyro data (32 data points) for one axis from the circular buffer into fftData[i]
//   and applies the window to the edge values.
// Calculation steps 1 and 2 then calculate the fft output (32 data points) and put that back into the same fftData[i] array.
// We then use fftData[i] array for frequency centre calculations for that axis

// Each FFT output bin has width fftSamplingRateHz/32, ie 41.65Hz per bin at 1333Hz
// Usable bandwidth is half this, ie 666Hz if fftSamplingRateHz is 1333Hz, i.e. bin 1 is 41.65hz, bin 2 83.3hz etc

#define DYN_NOTCH_SMOOTH_HZ        4
#define FFT_BIN_COUNT              (FFT_WINDOW_SIZE / 2) // 16  // TODO: Zero-padding -> doubling FFT resolution
#define DYN_NOTCH_CALC_TICKS       (XYZ_AXIS_COUNT * 4) // 4 steps per axis
#define DYN_NOTCH_OSD_MIN_THROTTLE 20

static FAST_DATA_ZERO_INIT arm_rfft_fast_instance_f32 fftInstance;
static FAST_DATA_ZERO_INIT float        fftData[FFT_WINDOW_SIZE];
static FAST_DATA_ZERO_INIT float        rfftData[FFT_WINDOW_SIZE];
static FAST_DATA_ZERO_INIT float        fftRms;
static FAST_DATA_ZERO_INIT linkedList_t fftPeaks; // list of uint8_t
static FAST_DATA_ZERO_INIT uint16_t     fftSamplingRateHz;
static FAST_DATA_ZERO_INIT float        fftResolution;
static FAST_DATA_ZERO_INIT uint8_t      fftStartBin;
static FAST_DATA_ZERO_INIT uint16_t     dynNotchMinHz;
static FAST_DATA_ZERO_INIT uint16_t     dynNotchMaxHz;
static FAST_DATA_ZERO_INIT uint16_t     dynNotchMaxFFT;
static FAST_DATA_ZERO_INIT float        smoothFactor;
static FAST_DATA_ZERO_INIT uint8_t      samples;
static FAST_DATA_ZERO_INIT float        window[FFT_WINDOW_SIZE]; // Window function, see https://en.wikipedia.org/wiki/Window_function

void gyroDataAnalyseInit(uint32_t targetLooptimeUs)
{
#ifdef USE_MULTI_GYRO
    static bool gyroAnalyseInitialized;
    if (gyroAnalyseInitialized) {
        return;
    }
    gyroAnalyseInitialized = true;
#endif

	arm_rfft_fast_init_f32(&fftInstance, FFT_WINDOW_SIZE);
	linkedListInit(&fftPeaks, sizeof(uint8_t));

    dynNotchMinHz = gyroConfig()->dyn_notch_min_hz;
    dynNotchMaxHz = MAX(2 * dynNotchMinHz, gyroConfig()->dyn_notch_max_hz);

    const int gyroLoopRateHz = lrintf((1.0f / targetLooptimeUs) * 1e6f);
    samples = MAX(1, gyroLoopRateHz / (2 * dynNotchMaxHz)); //600hz, 8k looptime, 13.333

    fftSamplingRateHz = gyroLoopRateHz / samples;
    // eg 8k, user max 600hz, int(8000/1200) = 6 (6.666), fftSamplingRateHz = 1333hz, range 666Hz
    // eg 4k, user max 600hz, int(4000/1200) = 3 (3.333), fftSamplingRateHz = 1333hz, range 666Hz
    // eg 2k, user max 600hz, int(2000/1200) = 1 (1.666) fftSamplingRateHz = 2000hz, range 1000Hz
    // eg 2k, user max 400hz, int(2000/800) = 2 (2.5) fftSamplingRateHz = 1000hz, range 500Hz
    // eg 1k, user max 600hz, int(1000/1200) = 1 (max(1,0.8333)) fftSamplingRateHz = 1000hz, range 500Hz
    // the upper limit of DN is always going to be Nyquist

    fftResolution = (float)fftSamplingRateHz / FFT_WINDOW_SIZE; // 41.65hz per bin for medium
    fftStartBin = MAX(2, lrintf(dynNotchMinHz / fftResolution)); // can't use bin 0 because it is DC.
    smoothFactor = 2 * M_PIf * DYN_NOTCH_SMOOTH_HZ / (gyroLoopRateHz / 12); // minimum PT1 k value

	// Periodic Hann-Window (best window for combination of small main lobe width and high sidelobe-falloff)
	for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
		window[i] = (0.5f - 0.5f * cosf(2.0f * M_PIf * i / FFT_WINDOW_SIZE));
    }
}

void gyroDataAnalyseStateInit(gyroAnalyseState_t *state, uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    gyroDataAnalyseInit(targetLooptimeUs);
    state->maxSampleCount = samples;
    state->maxSampleCountRcp = 1.0f / state->maxSampleCount;

	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		linkedListInit(&state->centerFreq[axis], sizeof(float));
		for (uint8_t n = 0; n < gyro.notchFilterDynCount; n++) {
			// any init value is fine, but evenly spreading centerFreq's over frequency range makes them stabilize quicker
			const float initFreq = (n + 0.5f) * (dynNotchMaxHz - dynNotchMinHz) / (float)gyro.notchFilterDynCount + dynNotchMinHz;
			linkedListPushBackFloat(&state->centerFreq[axis], initFreq);
		}
		// bandwidth of dynamic notch filters will be kept constant => notchQ = centerFreq / bandwidth
		state->filterBandwidth = gyro.notchFilterDynQ * fftResolution;
	}
}

void gyroDataAnalysePush(gyroAnalyseState_t *state, const int axis, const float sample)
{
    state->oversampledGyroAccumulator[axis] += sample;
}

static void gyroDataAnalyseUpdate(gyroAnalyseState_t *state);

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
FAST_CODE void gyroDataAnalyse(gyroAnalyseState_t *state)
{
   	// Once there is new data ready this will get set to true in gyroDataAnalyseUpdate()
	state->filterUpdate = false;

    // samples should have been pushed by `gyroDataAnalysePush`
    // if gyro sampling is > 1kHz, accumulate and average multiple gyro samples
    state->sampleCount++;

    if (state->sampleCount == state->maxSampleCount) {
        state->sampleCount = 0;

        // calculate mean value of accumulated samples
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            float sample = state->oversampledGyroAccumulator[axis] * state->maxSampleCountRcp;
            state->downsampledGyroData[axis][state->circularBufferIdx] = sample;
            if (axis == 0) {
                DEBUG_SET(DEBUG_FFT, 2, lrintf(sample));
            }

            state->oversampledGyroAccumulator[axis] = 0;
        }

        state->circularBufferIdx = (state->circularBufferIdx + 1) % FFT_WINDOW_SIZE;

        // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
        // recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
        // at 4kHz gyro loop rate this means 8kHz / 4 / 3 = 666Hz => update every 1.5ms
        // at 4kHz gyro loop rate this means 4kHz / 4 / 3 = 333Hz => update every 3ms
        state->updateTicks = DYN_NOTCH_CALC_TICKS;
    }

    // calculate FFT and get filters ready for update
    if (state->updateTicks > 0) {
        gyroDataAnalyseUpdate(state);
        --state->updateTicks;
    }
}

void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen, const float32_t *pCoef, uint16_t twidCoefModifier);
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable);
void arm_rms_f32(float32_t * pSrc, uint32_t blockSize, float32_t * pResult);

static void getPeaksFFT(float *fftData, linkedList_t *peaks, const float noise);
static float getMeanBin(const float *fftData, const uint8_t bin);

/*
 * Analyse gyro data
 */
static FAST_CODE_NOINLINE void gyroDataAnalyseUpdate(gyroAnalyseState_t *state)
{
    enum {
        STEP_ARM_CFFT_F32,
        STEP_BITREVERSAL,
        STEP_STAGE_RFFT_F32,
        STEP_ARM_CMPLX_MAG_F32,
        STEP_DETECT_PEAKS,
        STEP_UPDATE_FREQUENCIES,
        STEP_WINDOW,
        STEP_COUNT
    };

    arm_cfft_instance_f32 *Sint = &(fftInstance.Sint);

    uint32_t startTime = 0;
    if (debugMode == (DEBUG_FFT_TIME)) {
        startTime = micros();
    }

    DEBUG_SET(DEBUG_FFT_TIME, 0, state->updateStep);
    switch (state->updateStep) {
        case STEP_ARM_CFFT_F32:
        {
            switch (FFT_BIN_COUNT) {
            case 16:
                // 16us
                arm_cfft_radix8by2_f32(Sint, fftData);
                break;
            case 32:
                // 35us
                arm_cfft_radix8by4_f32(Sint, fftData);
                break;
            case 64:
                // 70us
                arm_radix8_butterfly_f32(fftData, FFT_BIN_COUNT, Sint->pTwiddle, 1);
                break;
            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_BITREVERSAL:
        {
            // 6us
            arm_bitreversal_32((uint32_t *)fftData, Sint->bitRevLength, Sint->pBitRevTable);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_STAGE_RFFT_F32:
        {
            // 14us
            // this does not work in place => fftData AND rfftData needed
            stage_rfft_f32(&fftInstance, fftData, rfftData);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_ARM_CMPLX_MAG_F32:
        {
            // 8us
            arm_cmplx_mag_f32(rfftData, fftData, FFT_BIN_COUNT);
            DEBUG_SET(DEBUG_FFT_TIME, 2, micros() - startTime);
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_DETECT_PEAKS:
        {
			// 1us@F722
			// Calc RMS (root-mean-square) over dynamic notch range for noise estimation
			arm_rms_f32(&fftData[fftStartBin], (FFT_BIN_COUNT - fftStartBin), &fftRms);		

			// 6us@F722
			// Get FFT peaks of current axis
			getPeaksFFT(fftData, &fftPeaks, fftRms);
			state->filterUpdateCount = MIN(fftPeaks.size, gyro.notchFilterDynCount);

			// --us@F722
			// Get n biggest peaks in ascending bin order
			// ...

			DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

			break;
        }
        case STEP_UPDATE_FREQUENCIES:
        {
			// --us@F722
			// Update notch center frequencies for current axis
			for (uint8_t i = 0; i < state->filterUpdateCount; i++) {

				// Pointer to current notch center frequency
				float *centerFreq = (float *)linkedListFind(&state->centerFreq[state->updateAxis], i)->data;

				// 3us@F722
				// Calculate frequency corresponding to current FFT peak bin (freq = bin*res)
				const uint8_t peakBin = linkedListGetInt8(&fftPeaks, i);
				const float peakFreq = getMeanBin(fftData, peakBin) * fftResolution;

				// PT1 style smoothing for updating notch center frequencies of current axis
				*centerFreq += smoothFactor * (peakFreq - *centerFreq);
				
				if (calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
					dynNotchMaxFFT = MAX(dynNotchMaxFFT, *centerFreq);
				}
			}

			if (state->updateAxis == 0) {
				// DEBUG_SET(DEBUG_FFT, 3, lrintf(fftMeanIndex * 100));
				for (uint8_t i = 0; i < state->filterUpdateCount && i < 3; i++) {
					DEBUG_SET(DEBUG_FFT_FREQ, (i + 1), lrintf(linkedListGetFloat(&state->centerFreq[state->updateAxis], i)));
				}
				// DEBUG_SET(DEBUG_FFT_FREQ, 2, lrintf(dynamicFactor));
				// DEBUG_SET(DEBUG_DYN_LPF, 1, state->centerFreq[state->updateAxis]);
			}

			// If set to true, dynamic notch filters for current axis will get updated in gyro.c once
			state->filterUpdate = true;

            state->updateAxis = (state->updateAxis + 1) % XYZ_AXIS_COUNT;
            state->updateStep++;
            
			DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            
            FALLTHROUGH;
        }
        case STEP_WINDOW:
        {
            // 5us
            // apply window to gyro samples and store result in fftData[i] to be used in step 1 and 2 and 3
            const uint8_t ringBufIdx = FFT_WINDOW_SIZE - state->circularBufferIdx;
            arm_mult_f32(&state->downsampledGyroData[state->updateAxis][state->circularBufferIdx], &window[0], &fftData[0], ringBufIdx);
            if (state->circularBufferIdx > 0) {
                arm_mult_f32(&state->downsampledGyroData[state->updateAxis][0], &window[ringBufIdx], &fftData[ringBufIdx], state->circularBufferIdx);
            }

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
        }
    }

    state->updateStep = (state->updateStep + 1) % STEP_COUNT;
}

static FAST_CODE void getPeaksFFT(float *fftData, linkedList_t *peaks, const float noise)
{
	// Get list ready for insertion
	linkedListClear(peaks);

	// Search for peaks and pits
	for (uint8_t bin = fftStartBin; bin < (FFT_BIN_COUNT - 1); bin++) {

		if (peaks != NULL) {
			const bool maximum = (fftData[bin] > fftData[bin-1]) && (fftData[bin] > fftData[bin+1]);
			if (maximum) {
				const bool significant = fftData[bin] * fftData[bin] > noise * noise / bin;
				if (significant) {
					linkedListPushBackInt8(peaks, bin);
				}
				bin++; // If bin is peak, next bin can't be peak
			}
		}
	}
}

// fftMeanIndex calculation is derived from fitting a gaussian function f(x) = Exp(ax²+bx+c) over peak and
//   shoulder points, then solving for position of maximum aka. meanBin (df/dx == 0 solved for x).
// When using a Hann-Window (best window for combination of small main lobe width and high sidelobe-falloff)
//  - MEAN	  error: 1.13% of fftResolution
//  - MAXIMUM error: 1.60% of fftResolution (when meanBin = bin-0.25 or bin+0.25)
//  - MINIMUM error: exactly 0 (when meanBin = bin-0.5 or bin or bin+0.5)
static FAST_CODE float getMeanBin(const float *fftData, const uint8_t bin)
{
	const float y0 = fftData[bin - 1];
	const float y1 = fftData[bin];
	const float y2 = fftData[bin + 1];

	//// 1us@F722
	//if (y0 > 0.0f && y1 > 0.0f && y2 > 0.0f) {
	//	y0 = log_approx(y0);
	//	y1 = log_approx(y1);
	//	y2 = log_approx(y2);
	//}

	const float denom = 2.0f * (y0 - 2 * y1 + y2);

	if (denom != 0.0f) {
		return bin + (y0 - y2) / denom;
	} else {
		return bin;
	}
}

uint16_t getMaxFFT(void) {
    return dynNotchMaxFFT;
}

void resetMaxFFT(void) {
    dynNotchMaxFFT = 0;
}

#endif // USE_GYRO_DATA_ANALYSE
