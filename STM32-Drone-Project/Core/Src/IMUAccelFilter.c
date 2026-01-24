#include "IMUAccelFilter.h"
#include <math.h>

static inline float lpf_alpha(float dt, float cutoff_hz)
{
    const float RC = 1.0f / (2.0f * 3.1415926535f * cutoff_hz);
    return dt / (dt + RC);
}

static inline void lpf3(float dt, float cutoff_hz,
                        float ax, float ay, float az,
                        float *ox, float *oy, float *oz)
{
    float a = lpf_alpha(dt, cutoff_hz);
    *ox += a * (ax - *ox);
    *oy += a * (ay - *oy);
    *oz += a * (az - *oz);
}

void ImuAccelFilter_Init(ImuAccelFilter_t* f, float cutoff_hz, float g_min, float g_max)
{
    f->ax_f = 0.0f;
    f->ay_f = 0.0f;
    f->az_f = 1.0f;
    f->cutoff_hz = cutoff_hz;
    f->g_min = g_min;
    f->g_max = g_max;
}

int ImuAccelFilter_Update(ImuAccelFilter_t* f, float ax, float ay, float az, float dt,
                          float* out_ax, float* out_ay, float* out_az)
{
    lpf3(dt, f->cutoff_hz, ax, ay, az, &f->ax_f, &f->ay_f, &f->az_f);

    *out_ax = f->ax_f;
    *out_ay = f->ay_f;
    *out_az = f->az_f;

    float n = sqrtf(f->ax_f*f->ax_f + f->ay_f*f->ay_f + f->az_f*f->az_f);
    return (n >= f->g_min && n <= f->g_max) ? 1 : 0;
}
