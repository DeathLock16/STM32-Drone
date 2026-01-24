#pragma once

typedef struct
{
    float ax_f, ay_f, az_f;
    float cutoff_hz;
    float g_min;
    float g_max;
} ImuAccelFilter_t;

void ImuAccelFilter_Init(ImuAccelFilter_t* f, float cutoff_hz, float g_min, float g_max);

int ImuAccelFilter_Update(ImuAccelFilter_t* f, float ax, float ay, float az, float dt,
                          float* out_ax, float* out_ay, float* out_az);
