#include "filter.h"
#include "mpu6050.h"
#include "esp_timer.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*-----------------------------------------------------------------------
 * Configuration
 *---------------------------------------------------------------------*/
#define DEFAULT_SAMPLE_RATE_HZ  100.0f   // Hz, used for very first dt
#define LPF_CUTOFF_HZ             25.0f  // Hz, tune as needed

/*-----------------------------------------------------------------------
 * LPF state per axis
 *---------------------------------------------------------------------*/
typedef struct {
    bool  init;        // first sample stored?
    float prev;        // last output
} LpfAxis;

static struct {
    LpfAxis x, y, z;   // filter state for ax, ay, az
    int64_t last_us;   // timestamp of previous update
} lpf;

/*-----------------------------------------------------------------------
 * Euler cache (roll, pitch from gravity)
 *---------------------------------------------------------------------*/
static euler_angles_t cached_euler;

/*-----------------------------------------------------------------------
 * Helper – apply first-order LPF to one axis
 *---------------------------------------------------------------------*/
static inline float lpf_axis_apply(LpfAxis *a,
                                   float    sample,
                                   float    alpha)
{
    if (!a->init) {                 // first sample: no lag
        a->prev = sample;
        a->init = true;
    }
    a->prev += alpha * (sample - a->prev);
    return a->prev;
}

/*-----------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------*/
void filter_init(void)
{
    lpf.x.init = lpf.y.init = lpf.z.init = false;
    lpf.last_us = 0;

    cached_euler.roll  = 0.0f;
    cached_euler.pitch = 0.0f;
    cached_euler.yaw   = 0.0f;
}

void filter_update(const imu_data_t *imu)
{
    /* ---------------------------------------------------------------
     * 1. Work out dt (seconds) for this frame
     * ------------------------------------------------------------- */
    const int64_t now_us = esp_timer_get_time();
    float dt;

    if (lpf.last_us == 0) {
        dt = 1.0f / DEFAULT_SAMPLE_RATE_HZ;        // first call fallback
    } else {
        dt = (now_us - lpf.last_us) / 1e6f;        // µs → s
        if (dt <= 0.0f) dt = 1.0f / DEFAULT_SAMPLE_RATE_HZ;
    }
    lpf.last_us = now_us;

    /* ---------------------------------------------------------------
     * 2. Compute α for this dt and cut-off Fc
     * ------------------------------------------------------------- */
    const float RC    = 1.0f / (2.0f * M_PI * LPF_CUTOFF_HZ);
    const float alpha = dt / (RC + dt);            // 0 < α < 1

    /* ---------------------------------------------------------------
     * 3. Low-pass each accelerometer axis
     * ------------------------------------------------------------- */
    const float ax_f = lpf_axis_apply(&lpf.x, imu->ax, alpha);
    const float ay_f = lpf_axis_apply(&lpf.y, imu->ay, alpha);
    const float az_f = lpf_axis_apply(&lpf.z, imu->az, alpha);

    /* ---------------------------------------------------------------
     * 4. Roll & pitch from gravity (yaw indeterminate)
     * ------------------------------------------------------------- */
    cached_euler.roll  = atan2f(ay_f,  az_f) *
                         180.0f / (float)M_PI;
    cached_euler.pitch = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) *
                         180.0f / (float)M_PI;
    cached_euler.yaw   = 0.0f;                     // accel cannot give yaw
}

euler_angles_t filter_get_euler(void)
{
    return cached_euler;   // already in degrees
}
