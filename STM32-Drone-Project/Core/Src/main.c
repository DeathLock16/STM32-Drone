/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "Madgwick.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    CTRL_MANUAL = 0,
    CTRL_STAB   = 1
} ControlMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG2RAD 0.0174532925f
#define RAD2DEG 57.2957795f

#define IMU_UPDATE_PERIOD_MS 10u

#define ACC_LPF_CUTOFF_HZ  20.0f
#define ACC_G_MIN          0.80f
#define ACC_G_MAX          1.20f

#define TILT_KILL_DEG      35.0f
#define TILT_UNKILL_DEG    25.0f
#define TILT_KILL_DEBOUNCE_MS   50u
#define TILT_UNKILL_HOLD_MS    300u

#define PWM_MIN 0
#define PWM_MAX 10000
#define PWM_STAB_FLOOR 500

#define STAB_KP_ROLL   25.0f
#define STAB_KD_ROLL    2.0f
#define STAB_KP_PITCH  25.0f
#define STAB_KD_PITCH   2.0f

/* yaw: podbite żeby było widać efekt */
#define STAB_KP_YAW_RATE  8.0f
#define STAB_KFF_YAW      2.0f
#define STAB_YAW_SIGN     1.0f
#define STAB_YAW_MAX      1800.0f

#define STAB_ROLL_SIGN    1.0f
#define STAB_PITCH_SIGN   1.0f
#define STAB_U_MAX        1400.0f

/* NAV */
#define NAV_TILT_DEG        10.0f
#define NAV_YAW_RATE_DPS   120.0f
#define NAV_THRUST_BIAS     700

/* Płynność (slew-rate) */
#define CMD_TILT_SLEW_DEG_PER_S      80.0f
#define CMD_YAW_SLEW_DPS_PER_S      600.0f
#define BASE_SLEW_PWM_PER_S       25000.0f
#define NAV_BIAS_SLEW_PWM_PER_S    6000.0f

/* mnożniki silników */
#define M_LF 1.00f
#define M_RF 0.92f
#define M_LB 1.00f
#define M_RB 1.00f

#define YAW_TRIM  0.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t imu;
Madgwick_t filter;

static uint32_t lastTick = 0;

static ImuPayload_t g_imu_last;
static uint8_t g_imu_ready = 0;
static uint32_t g_imu_last_update = 0;

/* tilt-kill */
static volatile uint8_t g_tilt_kill = 0;
static uint32_t g_tilt_kill_since = 0;
static uint32_t g_tilt_unkill_since = 0;

/* ACC LPF state */
static float ax_f = 0.0f, ay_f = 0.0f, az_f = 1.0f;

/* last pwm */
static PwmPayload_t g_last_pwm_cmd;
static uint8_t g_have_pwm_cmd = 0;

static volatile ControlMode_t g_ctrl_mode = CTRL_MANUAL;
static volatile uint8_t g_armed = 0;

/* attitude */
static volatile float g_roll_deg = 0.0f;
static volatile float g_pitch_deg = 0.0f;
static volatile float g_gyro_roll_dps = 0.0f;
static volatile float g_gyro_pitch_dps = 0.0f;
static volatile float g_gyro_yaw_dps = 0.0f;
static volatile uint8_t g_att_valid = 0;

/* aktualne komendy (płynnie dobijane do targetów) */
static volatile float g_cmd_roll_deg = 0.0f;
static volatile float g_cmd_pitch_deg = 0.0f;
static volatile float g_cmd_yaw_rate_dps = 0.0f;
static volatile uint16_t g_stab_base_pwm = 0;
static volatile int32_t g_nav_bias = 0;

/* targety (ustawiane ramką) */
static volatile float g_target_cmd_roll_deg = 0.0f;
static volatile float g_target_cmd_pitch_deg = 0.0f;
static volatile float g_target_cmd_yaw_rate_dps = 0.0f;
static volatile uint16_t g_target_base_pwm = 0;
static volatile uint8_t g_last_nav_action = NAV_STOP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#define UART_RX_RING_SIZE 256

static volatile uint8_t  uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;
static uint8_t uart_rx_it_byte;

static void UartRing_Push(uint8_t b)
{
    uint16_t next = (uint16_t)((uart_rx_head + 1) % UART_RX_RING_SIZE);
    if (next != uart_rx_tail)
    {
        uart_rx_ring[uart_rx_head] = b;
        uart_rx_head = next;
    }
}

static int UartRing_Pop(uint8_t *out)
{
    if (uart_rx_tail == uart_rx_head)
        return 0;

    *out = uart_rx_ring[uart_rx_tail];
    uart_rx_tail = (uint16_t)((uart_rx_tail + 1) % UART_RX_RING_SIZE);
    return 1;
}

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

static inline int accel_norm_ok(float ax, float ay, float az, float g_min, float g_max)
{
    float n = sqrtf(ax*ax + ay*ay + az*az);
    return (n >= g_min && n <= g_max) ? 1 : 0;
}

static inline uint16_t clamp_u16(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (uint16_t)lo;
    if (v > hi) return (uint16_t)hi;
    return (uint16_t)v;
}

static inline float clamp_f(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int32_t i32_min4(int32_t a, int32_t b, int32_t c, int32_t d)
{
    int32_t m = a;
    if (b < m) m = b;
    if (c < m) m = c;
    if (d < m) m = d;
    return m;
}

static inline int32_t i32_max4(int32_t a, int32_t b, int32_t c, int32_t d)
{
    int32_t m = a;
    if (b > m) m = b;
    if (c > m) m = c;
    if (d > m) m = d;
    return m;
}

/* Przesuwa wszystkie 4 kanały tak, aby zachować różnice (mix), a jednocześnie nie wpaść w floor/max */
static void Pwm_KeepHeadroom(int32_t* lf, int32_t* rf, int32_t* lb, int32_t* rb)
{
    int32_t mn = i32_min4(*lf, *rf, *lb, *rb);
    if (mn < PWM_STAB_FLOOR)
    {
        int32_t d = (int32_t)PWM_STAB_FLOOR - mn;
        *lf += d; *rf += d; *lb += d; *rb += d;
    }

    int32_t mx = i32_max4(*lf, *rf, *lb, *rb);
    if (mx > PWM_MAX)
    {
        int32_t d = mx - (int32_t)PWM_MAX;
        *lf -= d; *rf -= d; *lb -= d; *rb -= d;
    }
}

/* slew-rate (płynne dochodzenie do targetu) */
static inline float slew_f(float cur, float target, float max_rate_per_s, float dt)
{
    float max_step = max_rate_per_s * dt;
    float d = target - cur;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    return cur + d;
}

static inline int32_t slew_i32(int32_t cur, int32_t target, float max_rate_per_s, float dt)
{
    float max_step_f = max_rate_per_s * dt;
    int32_t max_step = (int32_t)lroundf(max_step_f);
    int32_t d = target - cur;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    return cur + d;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Redirect printf to USART6 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart6, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

static void ESP_LeaveSendMode(void)
{
    const char* cmd = "+";
    HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_Delay(1000);
}

static void ESP_Send(const char *cmd, uint32_t delay_ms)
{
    HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    HAL_Delay(delay_ms);
}

static int ESP_WaitFor(const char *expected, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint16_t idx = 0;
    char buf[64] = {0};
    uint8_t c;

    while (HAL_GetTick() - t0 < timeout_ms)
    {
        if (HAL_UART_Receive(&huart6, &c, 1, 10) == HAL_OK)
        {
            if (idx < sizeof(buf) - 1)
            {
                buf[idx++] = (char)c;
                buf[idx] = 0;
            }

            if (strstr(buf, expected))
                return 1;
        }
    }
    return 0;
}

static void ESP_ConnectWithRetry(void)
{
    while (1)
    {
        ESP_Send("AT+CIPSTART=\"TCP\",\"192.168.8.100\",3333", 0);

        if (ESP_WaitFor("OK", 3000) ||
            ESP_WaitFor("ALREADY CONNECTED", 1000))
        {
            break;
        }

        HAL_Delay(500);
        HAL_Delay(500);
    }
}

void ESP_Init(void)
{
    ESP_LeaveSendMode();
    ESP_Send("AT+RST", 2000);
    ESP_Send("ATE0", 200);

    ESP_Send("AT+CWMODE=1", 200);
    ESP_Send("AT+CIPSTA=\"192.168.8.127\",\"192.168.8.1\",\"255.255.255.0\"", 500);

    ESP_Send("AT+CWJAP=\"HUAWEI-7975\",\"40720250\"", 500);

    ESP_Send("AT+CIPMUX=0", 200);
    ESP_ConnectWithRetry();

    ESP_Send("AT+CIPMODE=1", 200);
    ESP_Send("AT+CIPSEND", 0);

    if (!ESP_WaitFor(">", 2000))
    {
        while (1) { }
    }

    uint8_t dump;
    while (HAL_UART_Receive(&huart6, &dump, 1, 10) == HAL_OK)
    {
    }
}

static void IMU_Init(void)
{
    if (MPU6050_Init(&hi2c1) != HAL_OK)
        Error_Handler();

    if (MPU6050_CalibrateGyro(&hi2c1, &imu, 1000) != HAL_OK)
        Error_Handler();

    Madgwick_Init(&filter, 0.05f);

    filter.q0 = 1.0f;
    filter.q1 = 0.0f;
    filter.q2 = 0.0f;
    filter.q3 = 0.0f;

    ax_f = 0.0f; ay_f = 0.0f; az_f = 1.0f;

    lastTick = HAL_GetTick();
    g_imu_last_update = lastTick;

    g_tilt_kill = 0;
    g_tilt_kill_since = 0;
    g_tilt_unkill_since = 0;

    g_cmd_roll_deg = 0.0f;
    g_cmd_pitch_deg = 0.0f;
    g_cmd_yaw_rate_dps = 0.0f;
    g_target_cmd_roll_deg = 0.0f;
    g_target_cmd_pitch_deg = 0.0f;
    g_target_cmd_yaw_rate_dps = 0.0f;

    g_stab_base_pwm = 0;
    g_target_base_pwm = 0;

    g_nav_bias = 0;
    g_last_nav_action = NAV_STOP;

    HAL_GPIO_WritePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin, 0);
}

static void PWM_StartAll(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim1);
}

static void PWM_SetSafe(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

static void ReadPwm(PwmPayload_t *p)
{
    p->motor_lb = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    p->motor_lf = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    p->motor_rf = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    p->motor_rb = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
}

static void SetPwm(const PwmPayload_t *p)
{
    if (!g_armed) g_armed = 1;

    g_last_pwm_cmd = *p;
    g_have_pwm_cmd = 1;

    if (g_tilt_kill)
    {
        PWM_SetSafe();
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p->motor_lb);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p->motor_lf);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p->motor_rf);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, p->motor_rb);
}

static void IMU_UpdateContinuous(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - g_imu_last_update) < IMU_UPDATE_PERIOD_MS)
        return;

    g_imu_last_update = now;

    MPU6050_ReadRaw(&hi2c1, &imu);
    MPU6050_ComputeScaled(&imu);

    if (lastTick == 0)
    {
        lastTick = now;
        return;
    }

    float dt = (now - lastTick) * 0.001f;
    lastTick = now;

    if (dt <= 0.0f)
        return;

    if (dt > 0.02f)
        dt = 0.02f;

    lpf3(dt, ACC_LPF_CUTOFF_HZ, imu.acc_x, imu.acc_y, imu.acc_z, &ax_f, &ay_f, &az_f);
    int acc_ok = accel_norm_ok(ax_f, ay_f, az_f, ACC_G_MIN, ACC_G_MAX);

    float gx = imu.gyro_x * DEG2RAD;
    float gy = imu.gyro_y * DEG2RAD;
    float gz = imu.gyro_z * DEG2RAD;

    if (acc_ok)
        Madgwick_UpdateIMU(&filter, gx, gy, gz, ax_f, ay_f, az_f, dt);
    else
        Madgwick_UpdateIMU(&filter, gx, gy, gz, 0.0f, 0.0f, 0.0f, dt);

    float roll  = atan2f(
        2.0f * (filter.q0 * filter.q1 + filter.q2 * filter.q3),
        1.0f - 2.0f * (filter.q1 * filter.q1 + filter.q2 * filter.q2)
    ) * RAD2DEG;

    float pitch = asinf(
        2.0f * (filter.q0 * filter.q2 - filter.q3 * filter.q1)
    ) * RAD2DEG;

    float yaw = atan2f(
        2.0f * (filter.q0 * filter.q3 + filter.q1 * filter.q2),
        1.0f - 2.0f * (filter.q2 * filter.q2 + filter.q3 * filter.q3)
    ) * RAD2DEG;

    g_imu_last.roll  = (int16_t)(roll  * 100.0f);
    g_imu_last.pitch = (int16_t)(pitch * 100.0f);
    g_imu_last.yaw   = (int16_t)(yaw   * 100.0f);

    g_roll_deg = roll;
    g_pitch_deg = pitch;

    g_gyro_roll_dps = imu.gyro_x;
    g_gyro_pitch_dps = imu.gyro_y;
    g_gyro_yaw_dps = imu.gyro_z;

    g_att_valid = 1;

    float aroll = fabsf(roll);
    float apitch = fabsf(pitch);

    if (!g_tilt_kill)
    {
        if (aroll > TILT_KILL_DEG || apitch > TILT_KILL_DEG)
        {
            if (g_tilt_kill_since == 0)
                g_tilt_kill_since = now;

            if ((now - g_tilt_kill_since) >= TILT_KILL_DEBOUNCE_MS)
            {
                g_tilt_kill = 1;
                g_tilt_unkill_since = 0;
                HAL_GPIO_WritePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin, 1);
                PWM_SetSafe();
                g_ctrl_mode = CTRL_MANUAL;
            }
        }
        else
        {
            g_tilt_kill_since = 0;
        }
    }
    else
    {
        if (aroll < TILT_UNKILL_DEG && apitch < TILT_UNKILL_DEG)
        {
            if (g_tilt_unkill_since == 0)
                g_tilt_unkill_since = now;

            if ((now - g_tilt_unkill_since) >= TILT_UNKILL_HOLD_MS)
            {
                g_tilt_kill = 0;
                g_tilt_kill_since = 0;
                HAL_GPIO_WritePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin, 0);
            }
        }
        else
        {
            g_tilt_unkill_since = 0;
        }
    }

    g_imu_ready = 1;
}

static void ControlStep_Stabilize(void)
{
    static uint32_t last_ctrl = 0;
    uint32_t now = HAL_GetTick();
    float dt = 0.01f;

    if (last_ctrl != 0)
    {
        dt = (now - last_ctrl) * 0.001f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f)  dt = 0.05f;
    }
    last_ctrl = now;

    if (g_tilt_kill)
    {
        PWM_SetSafe();
        return;
    }

    if (!g_att_valid)
        return;

    /* PŁYNNE DOJŚCIE DO TARGETÓW */
    g_stab_base_pwm = (uint16_t)lroundf(
        slew_f((float)g_stab_base_pwm, (float)g_target_base_pwm, BASE_SLEW_PWM_PER_S, dt)
    );

    g_cmd_roll_deg = slew_f(g_cmd_roll_deg, g_target_cmd_roll_deg, CMD_TILT_SLEW_DEG_PER_S, dt);
    g_cmd_pitch_deg = slew_f(g_cmd_pitch_deg, g_target_cmd_pitch_deg, CMD_TILT_SLEW_DEG_PER_S, dt);
    g_cmd_yaw_rate_dps = slew_f(g_cmd_yaw_rate_dps, g_target_cmd_yaw_rate_dps, CMD_YAW_SLEW_DPS_PER_S, dt);

    int32_t target_bias = 0;
    if (g_last_nav_action == NAV_FORWARD || g_last_nav_action == NAV_BACK ||
        g_last_nav_action == NAV_LEFT    || g_last_nav_action == NAV_RIGHT)
    {
        target_bias = (int32_t)NAV_THRUST_BIAS;
    }
    g_nav_bias = slew_i32(g_nav_bias, target_bias, NAV_BIAS_SLEW_PWM_PER_S, dt);

    float roll_meas  = (float)g_roll_deg  * STAB_ROLL_SIGN;
    float pitch_meas = (float)g_pitch_deg * STAB_PITCH_SIGN;

    float roll_rate  = (float)g_gyro_roll_dps  * STAB_ROLL_SIGN;
    float pitch_rate = (float)g_gyro_pitch_dps * STAB_PITCH_SIGN;

    float err_roll  = g_cmd_roll_deg  - roll_meas;
    float err_pitch = g_cmd_pitch_deg - pitch_meas;

    float u_roll  = STAB_KP_ROLL  * err_roll  - STAB_KD_ROLL  * roll_rate;
    float u_pitch = STAB_KP_PITCH * err_pitch - STAB_KD_PITCH * pitch_rate;

    u_roll  = clamp_f(u_roll,  -STAB_U_MAX, STAB_U_MAX);
    u_pitch = clamp_f(u_pitch, -STAB_U_MAX, STAB_U_MAX);

    float yaw_rate = (float)g_gyro_yaw_dps * STAB_YAW_SIGN;
    float yaw_cmd  = (float)g_cmd_yaw_rate_dps * STAB_YAW_SIGN;

    float yaw_rate_err = yaw_cmd - yaw_rate;

    float u_yaw = STAB_KP_YAW_RATE * yaw_rate_err + STAB_KFF_YAW * yaw_cmd;
    u_yaw = clamp_f(u_yaw, -STAB_YAW_MAX, STAB_YAW_MAX);

    float u_yaw_total = u_yaw + YAW_TRIM;
    u_yaw_total = clamp_f(u_yaw_total, -STAB_YAW_MAX, STAB_YAW_MAX);

    int32_t base = (int32_t)g_stab_base_pwm;

    int32_t lf = base + (int32_t)lroundf(u_pitch + u_roll - u_yaw_total);
    int32_t rf = base + (int32_t)lroundf(u_pitch - u_roll + u_yaw_total);
    int32_t lb = base + (int32_t)lroundf(-u_pitch + u_roll + u_yaw_total);
    int32_t rb = base + (int32_t)lroundf(-u_pitch - u_roll - u_yaw_total);

    int32_t nav = g_nav_bias;

    switch ((uint8_t)g_last_nav_action)
    {
        case NAV_FORWARD:
            lb += nav; rb += nav;
            lf -= nav; rf -= nav;
            break;

        case NAV_BACK:
            lb -= nav; rb -= nav;
            lf += nav; rf += nav;
            break;

        case NAV_LEFT:
            rb += nav; rf += nav;
            lb -= nav; lf -= nav;
            break;

        case NAV_RIGHT:
            lb += nav; lf += nav;
            rb -= nav; rf -= nav;
            break;

        default:
            break;
    }

    Pwm_KeepHeadroom(&lf, &rf, &lb, &rb);

    int32_t lf_m = (int32_t)lroundf((float)lf * M_LF);
    int32_t rf_m = (int32_t)lroundf((float)rf * M_RF);
    int32_t lb_m = (int32_t)lroundf((float)lb * M_LB);
    int32_t rb_m = (int32_t)lroundf((float)rb * M_RB);

    PwmPayload_t out;
    out.motor_lf = clamp_u16(lf_m, PWM_STAB_FLOOR, PWM_MAX);
    out.motor_rf = clamp_u16(rf_m, PWM_STAB_FLOOR, PWM_MAX);
    out.motor_lb = clamp_u16(lb_m, PWM_STAB_FLOOR, PWM_MAX);
    out.motor_rb = clamp_u16(rb_m, PWM_STAB_FLOOR, PWM_MAX);

    SetPwm(&out);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
  PWM_StartAll();
  PWM_SetSafe();

  ESP_Init();
  IMU_Init();
  Protocol_Init();

  uart_rx_head = 0;
  uart_rx_tail = 0;

  HAL_UART_AbortReceive(&huart6);
  __HAL_UART_CLEAR_OREFLAG(&huart6);
  HAL_UART_Receive_IT(&huart6, &uart_rx_it_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    IMU_UpdateContinuous();

    if (!g_armed || g_tilt_kill)
      PWM_SetSafe();
    else if (g_ctrl_mode == CTRL_STAB)
      ControlStep_Stabilize();

    uint8_t b;
    while (UartRing_Pop(&b))
    {
      ProtoResult_t r = Protocol_PushByte(b);

      if (r == PROTO_OK)
      {
        const ProtoRxFrame_t *rx = Protocol_GetFrame();

        uint8_t tx[64];
        uint8_t tx_len = 0;

        if (rx->cmd == CMD_PING)
        {
          tx_len = Protocol_BuildFrame(DIR_STM_TO_PC, CMD_PONG, NULL, 0, tx, sizeof(tx));
        }
        else if (rx->cmd == CMD_PWM_READ)
        {
          PwmPayload_t pwm;
          ReadPwm(&pwm);
          tx_len = Protocol_BuildFrame(DIR_STM_TO_PC, CMD_PWM_DATA,
                                       (const uint8_t*)&pwm, PWM_PAYLOAD_SIZE,
                                       tx, sizeof(tx));
        }
        else if (rx->cmd == CMD_PWM_SET)
        {
          if (rx->len != PWM_PAYLOAD_SIZE)
          {
            tx_len = Protocol_BuildStatusFrame(ST_ERR_BAD_LEN, tx, sizeof(tx));
          }
          else
          {
            PwmPayload_t pwm;
            memcpy(&pwm, rx->data, PWM_PAYLOAD_SIZE);

            g_ctrl_mode = CTRL_MANUAL;

            g_target_cmd_roll_deg = 0.0f;
            g_target_cmd_pitch_deg = 0.0f;
            g_target_cmd_yaw_rate_dps = 0.0f;

            g_cmd_roll_deg = 0.0f;
            g_cmd_pitch_deg = 0.0f;
            g_cmd_yaw_rate_dps = 0.0f;

            g_last_nav_action = NAV_STOP;
            g_nav_bias = 0;

            SetPwm(&pwm);
            tx_len = Protocol_BuildFrame(DIR_STM_TO_PC, CMD_PWM_ACK, NULL, 0, tx, sizeof(tx));
          }
        }
        else if (rx->cmd == CMD_IMU_READ)
        {
          if (!g_imu_ready)
          {
            tx_len = Protocol_BuildStatusFrame(ST_ERR_IMU_NOT_READY, tx, sizeof(tx));
          }
          else
          {
            ImuPayload_t snap = g_imu_last;
            tx_len = Protocol_BuildFrame(DIR_STM_TO_PC, CMD_IMU_DATA,
                                         (const uint8_t*)&snap, IMU_PAYLOAD_SIZE,
                                         tx, sizeof(tx));
          }
        }
        else if (rx->cmd == CMD_STAB_SET)
        {
          if (rx->len != STAB_PAYLOAD_SIZE)
          {
            tx_len = Protocol_BuildStatusFrame(ST_ERR_BAD_LEN, tx, sizeof(tx));
          }
          else
          {
            StabPayload_t sp;
            memcpy(&sp, rx->data, STAB_PAYLOAD_SIZE);

            if (sp.base_pwm > PWM_MAX) sp.base_pwm = PWM_MAX;

            g_target_base_pwm = sp.base_pwm;
            g_ctrl_mode = CTRL_STAB;
            g_armed = 1;

            g_target_cmd_roll_deg = 0.0f;
            g_target_cmd_pitch_deg = 0.0f;
            g_target_cmd_yaw_rate_dps = 0.0f;

            g_last_nav_action = NAV_STOP;

            tx_len = Protocol_BuildFrame(DIR_STM_TO_PC, CMD_STAB_ACK,
                                         (const uint8_t*)&sp, STAB_PAYLOAD_SIZE,
                                         tx, sizeof(tx));
          }
        }
        else if (rx->cmd == CMD_STAB_OFF)
        {
          g_ctrl_mode = CTRL_MANUAL;
          g_last_nav_action = NAV_STOP;
          g_nav_bias = 0;
          PWM_SetSafe();
          tx_len = Protocol_BuildFrame(DIR_STM_TO_PC, CMD_STAB_OFF_ACK, NULL, 0, tx, sizeof(tx));
        }
        else if (rx->cmd == CMD_NAV_SET)
        {
          if (rx->len != NAV_PAYLOAD_SIZE)
          {
            tx_len = Protocol_BuildStatusFrame(ST_ERR_BAD_LEN, tx, sizeof(tx));
          }
          else
          {
            NavPayload_t np;
            memcpy(&np, rx->data, NAV_PAYLOAD_SIZE);

            if (np.base_pwm > PWM_MAX) np.base_pwm = PWM_MAX;

            g_target_base_pwm = np.base_pwm;
            g_ctrl_mode = CTRL_STAB;
            g_armed = 1;

            g_target_cmd_roll_deg = 0.0f;
            g_target_cmd_pitch_deg = 0.0f;
            g_target_cmd_yaw_rate_dps = 0.0f;

            g_last_nav_action = np.action;

            switch ((uint8_t)np.action)
            {
              case NAV_STOP:
                break;

              case NAV_FORWARD:
                g_target_cmd_pitch_deg = -NAV_TILT_DEG;
                break;

              case NAV_BACK:
                g_target_cmd_pitch_deg = +NAV_TILT_DEG;
                break;

              case NAV_LEFT:
                g_target_cmd_roll_deg = -NAV_TILT_DEG;
                break;

              case NAV_RIGHT:
                g_target_cmd_roll_deg = +NAV_TILT_DEG;
                break;

              case NAV_YAW_LEFT:
                g_target_cmd_yaw_rate_dps = -NAV_YAW_RATE_DPS;
                break;

              case NAV_YAW_RIGHT:
                g_target_cmd_yaw_rate_dps = +NAV_YAW_RATE_DPS;
                break;

              default:
                break;
            }

            tx_len = Protocol_BuildFrame(DIR_STM_TO_PC, CMD_NAV_ACK,
                                         (const uint8_t*)&np, NAV_PAYLOAD_SIZE,
                                         tx, sizeof(tx));
          }
        }
        else
        {
          tx_len = Protocol_BuildStatusFrame(ST_ERR_UNKNOWN_CMD, tx, sizeof(tx));
        }

        if (tx_len > 0)
          HAL_UART_Transmit(&huart6, tx, tx_len, HAL_MAX_DELAY);
      }
      else if (r == PROTO_ERROR)
      {
        uint8_t tx[16];
        uint8_t tx_len = Protocol_BuildStatusFrame(Protocol_GetLastError(), tx, sizeof(tx));
        if (tx_len > 0)
          HAL_UART_Transmit(&huart6, tx, tx_len, HAL_MAX_DELAY);
      }
    }

    HAL_Delay(1);
    /* USER CODE END WHILE */
  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart6)
  {
    UartRing_Push(uart_rx_it_byte);
    HAL_UART_Receive_IT(&huart6, &uart_rx_it_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart6)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    __HAL_UART_CLEAR_NEFLAG(&huart6);
    __HAL_UART_CLEAR_FEFLAG(&huart6);
    __HAL_UART_CLEAR_PEFLAG(&huart6);

    HAL_UART_AbortReceive(&huart6);
    HAL_UART_Receive_IT(&huart6, &uart_rx_it_byte, 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
