#pragma once
#include <math.h>
#include <stdint.h>

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float beta;
} Madgwick_t;

typedef struct
{
    int16_t roll;   // *100 deg
    int16_t pitch;  // *100 deg
    int16_t yaw;    // *100 deg
} ImuAngles_t;

typedef enum
{
    IMU_OK = 0,
    IMU_SKIP,
    IMU_ERROR
} ImuResult_t;

void Madgwick_Init(Madgwick_t* m, float beta);

void Madgwick_UpdateIMU(
    Madgwick_t* m,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt
);

