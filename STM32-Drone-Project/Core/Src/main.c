/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#define STAB_KP_ROLL   12.0f
#define STAB_KD_ROLL   1.8f
#define STAB_KI_ROLL   0.0f

#define STAB_KP_PITCH  12.0f
#define STAB_KD_PITCH  1.8f
#define STAB_KI_PITCH  0.0f

#define STAB_KP_YAW_RATE  2.0f
#define STAB_KI_YAW_RATE  0.0f

#define STAB_I_MAX    200.0f

#define STAB_KFF_YAW      0.0f
#define STAB_YAW_SIGN     -1.0f
#define STAB_YAW_MAX      300.0f
#define STAB_YAW_I_MAX    200.0f

/* TWOJE OBSERWACJE:
   Prawo => dodatnie PITCH, Lewo => ujemne PITCH
   Tył   => dodatni  ROLL,  Przód => ujemny ROLL
   To oznacza: roll/pitch są zamienione względem osi drona (rotacja IMU o 90°).
*/
#define IMU_SWAP_ROLL_PITCH  1

/* Po swapie:
   - ROLL kontrolera (lewo/prawo) bierzemy z IMU_PITCH
   - PITCH kontrolera (przód/tył) bierzemy z IMU_ROLL
   Znaki wg Twoich obserwacji wychodzą dodatnie, więc startowo +1. */
#define STAB_ROLL_SIGN    1.0f
#define STAB_PITCH_SIGN   1.0f

#define STAB_U_MAX        1400.0f

#define NAV_TILT_DEG        10.0f
#define NAV_YAW_RATE_DPS   120.0f
#define NAV_THRUST_BIAS     0

#define CMD_TILT_SLEW_DEG_PER_S      80.0f
#define CMD_YAW_SLEW_DPS_PER_S      600.0f
#define BASE_SLEW_PWM_PER_S       25000.0f
#define NAV_BIAS_SLEW_PWM_PER_S    6000.0f

#define YAW_MIX_FRONT_CCW_REAR_CW  1

#define MOTOR_GAIN_LF   1.000f
#define MOTOR_GAIN_RF   1.000f
#define MOTOR_GAIN_LB   1.000f
#define MOTOR_GAIN_RB   1.000f

#define MOTOR_OFFS_LF   0.0f
#define MOTOR_OFFS_RF   0.0f
#define MOTOR_OFFS_LB   0.0f
#define MOTOR_OFFS_RB   0.0f

#define YAW_TRIM  0.0f

#define ROLL_TRIM_DEG   0.0f
#define PITCH_TRIM_DEG  0.0f

#define LEVEL_CALIB_SAMPLES        200u
#define LEVEL_CALIB_GYRO_MAX_DPS   2.5f
#define LEVEL_CALIB_ACC_OK_REQUIRED 1

#define CG_PITCH_BIAS_PWM   0
#define CG_ROLL_BIAS_PWM    0

#define CG_BIAS_START_PWM   5600u
#define CG_BIAS_FULL_PWM    7600u

#define I_ENABLE_BASE_PWM   6500u
#define I_DISABLE_BASE_PWM  6000u
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
MPU6050_t imu;
Madgwick_t filter;

static uint32_t lastTick = 0;

static ImuPayload_t g_imu_last;
static uint8_t g_imu_ready = 0;
static uint32_t g_imu_last_update = 0;

static volatile uint8_t g_tilt_kill = 0;
static uint32_t g_tilt_kill_since = 0;
static uint32_t g_tilt_unkill_since = 0;

static float ax_f = 0.0f, ay_f = 0.0f, az_f = 1.0f;

static PwmPayload_t g_last_pwm_cmd;
static uint8_t g_have_pwm_cmd = 0;

static volatile ControlMode_t g_ctrl_mode = CTRL_MANUAL;
static volatile uint8_t g_armed = 0;

static float i_roll = 0.0f, i_pitch = 0.0f, i_yaw = 0.0f;
static uint8_t g_i_enabled = 0;

static volatile float g_roll_deg = 0.0f;
static volatile float g_pitch_deg = 0.0f;
static volatile float g_gyro_roll_dps = 0.0f;
static volatile float g_gyro_pitch_dps = 0.0f;
static volatile float g_gyro_yaw_dps = 0.0f;
static volatile uint8_t g_att_valid = 0;

static volatile float g_cmd_roll_deg = 0.0f;
static volatile float g_cmd_pitch_deg = 0.0f;
static volatile float g_cmd_yaw_rate_dps = 0.0f;
static volatile uint16_t g_stab_base_pwm = 0;
static volatile int32_t g_nav_bias = 0;

static volatile float g_target_cmd_roll_deg = 0.0f;
static volatile float g_target_cmd_pitch_deg = 0.0f;
static volatile float g_target_cmd_yaw_rate_dps = 0.0f;
static volatile uint16_t g_target_base_pwm = 0;
static volatile uint8_t g_last_nav_action = NAV_STOP;

static volatile uint8_t g_imu_new = 0;

static volatile float g_level_roll_off = 0.0f;
static volatile float g_level_pitch_off = 0.0f;
static volatile uint8_t g_level_calib_done = 0;
static float g_level_sum_roll = 0.0f;
static float g_level_sum_pitch = 0.0f;
static uint16_t g_level_n = 0;
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
    if (uart_rx_tail == uart_rx_head) return 0;
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

static float Mixer_ComputeScale(int32_t base, float c_lf, float c_rf, float c_lb, float c_rb)
{
    float k = 1.0f;
    float c[4] = { c_lf, c_rf, c_lb, c_rb };

    for (int i = 0; i < 4; ++i)
    {
        float ci = c[i];
        if (ci > 0.0f)
        {
            float lim = ((float)PWM_MAX - (float)base) / ci;
            if (lim < k) k = lim;
        }
        else if (ci < 0.0f)
        {
            float lim = ((float)PWM_STAB_FLOOR - (float)base) / ci;
            if (lim < k) k = lim;
        }
    }

    if (k > 1.0f) k = 1.0f;
    if (k < 0.0f) k = 0.0f;
    return k;
}

static float CgBiasScale(uint16_t base_pwm)
{
    if (base_pwm <= CG_BIAS_START_PWM) return 0.0f;
    if (base_pwm >= CG_BIAS_FULL_PWM)  return 1.0f;
    return ((float)base_pwm - (float)CG_BIAS_START_PWM) /
           ((float)CG_BIAS_FULL_PWM  - (float)CG_BIAS_START_PWM);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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
            if (strstr(buf, expected)) return 1;
        }
    }
    return 0;
}

static void ESP_ConnectWithRetry(void)
{
    while (1)
    {
        ESP_Send("AT+CIPSTART=\"TCP\",\"192.168.8.100\",3333", 0);
        if (ESP_WaitFor("OK", 3000) || ESP_WaitFor("ALREADY CONNECTED", 1000)) break;
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

    if (!ESP_WaitFor(">", 2000)) { while (1) { } }

    uint8_t dump;
    while (HAL_UART_Receive(&huart6, &dump, 1, 10) == HAL_OK) { }
}

static void IMU_Init(void)
{
    if (MPU6050_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (MPU6050_CalibrateGyro(&hi2c1, &imu, 1000) != HAL_OK) Error_Handler();

    Madgwick_Init(&filter, 0.05f);
    filter.q0 = 1.0f; filter.q1 = 0.0f; filter.q2 = 0.0f; filter.q3 = 0.0f;

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

    i_roll = 0.0f;
    i_pitch = 0.0f;
    i_yaw = 0.0f;
    g_i_enabled = 0;

    g_imu_new = 0;

    g_level_roll_off = 0.0f;
    g_level_pitch_off = 0.0f;
    g_level_calib_done = 0;
    g_level_sum_roll = 0.0f;
    g_level_sum_pitch = 0.0f;
    g_level_n = 0;

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
    if ((now - g_imu_last_update) < IMU_UPDATE_PERIOD_MS) return;
    g_imu_last_update = now;

    MPU6050_ReadRaw(&hi2c1, &imu);
    MPU6050_ComputeScaled(&imu);

    if (lastTick == 0) { lastTick = now; return; }

    float dt = (now - lastTick) * 0.001f;
    lastTick = now;

    if (dt <= 0.0f) return;
    if (dt > 0.02f) dt = 0.02f;

    lpf3(dt, ACC_LPF_CUTOFF_HZ, imu.acc_x, imu.acc_y, imu.acc_z, &ax_f, &ay_f, &az_f);
    int acc_ok = accel_norm_ok(ax_f, ay_f, az_f, ACC_G_MIN, ACC_G_MAX);

    float gx = imu.gyro_x * DEG2RAD;
    float gy = imu.gyro_y * DEG2RAD;
    float gz = imu.gyro_z * DEG2RAD;

    if (acc_ok) Madgwick_UpdateIMU(&filter, gx, gy, gz, ax_f, ay_f, az_f, dt);
    else        Madgwick_UpdateIMU(&filter, gx, gy, gz, 0.0f, 0.0f, 0.0f, dt);

    float roll  = atan2f(
        2.0f * (filter.q0 * filter.q1 + filter.q2 * filter.q3),
        1.0f - 2.0f * (filter.q1 * filter.q1 + filter.q2 * filter.q2)
    ) * RAD2DEG;

    float s = 2.0f * (filter.q0 * filter.q2 - filter.q3 * filter.q1);
    s = clamp_f(s, -1.0f, 1.0f);
    float pitch = asinf(s) * RAD2DEG;

    float yaw = atan2f(
        2.0f * (filter.q0 * filter.q3 + filter.q1 * filter.q2),
        1.0f - 2.0f * (filter.q2 * filter.q2 + filter.q3 * filter.q3)
    ) * RAD2DEG;

    g_imu_last.roll  = (int16_t)(roll  * 100.0f);
    g_imu_last.pitch = (int16_t)(pitch * 100.0f);
    g_imu_last.yaw   = (int16_t)(yaw   * 100.0f);

    g_roll_deg = roll;
    g_pitch_deg = pitch;

    g_gyro_roll_dps  = imu.gyro_x;
    g_gyro_pitch_dps = imu.gyro_y;
    g_gyro_yaw_dps   = imu.gyro_z;

    g_att_valid = 1;

    if (!g_level_calib_done && !g_armed)
    {
        int gyro_ok = (fabsf(imu.gyro_x) < LEVEL_CALIB_GYRO_MAX_DPS) &&
                      (fabsf(imu.gyro_y) < LEVEL_CALIB_GYRO_MAX_DPS) &&
                      (fabsf(imu.gyro_z) < LEVEL_CALIB_GYRO_MAX_DPS);

        int acc_req_ok = 1;
#if LEVEL_CALIB_ACC_OK_REQUIRED
        acc_req_ok = acc_ok;
#endif

        if (gyro_ok && acc_req_ok)
        {
            g_level_sum_roll  += roll;
            g_level_sum_pitch += pitch;
            g_level_n++;

            if (g_level_n >= LEVEL_CALIB_SAMPLES)
            {
                g_level_roll_off  = g_level_sum_roll  / (float)g_level_n;
                g_level_pitch_off = g_level_sum_pitch / (float)g_level_n;
                g_level_calib_done = 1;

                HAL_GPIO_WritePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin, 1);
                HAL_Delay(60);
                HAL_GPIO_WritePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin, 0);
            }
        }
        else
        {
            g_level_sum_roll = 0.0f;
            g_level_sum_pitch = 0.0f;
            g_level_n = 0;
        }
    }

    float aroll  = fabsf(roll);
    float apitch = fabsf(pitch);

    if (!g_tilt_kill)
    {
        if (aroll > TILT_KILL_DEG || apitch > TILT_KILL_DEG)
        {
            if (g_tilt_kill_since == 0) g_tilt_kill_since = now;
            if ((now - g_tilt_kill_since) >= TILT_KILL_DEBOUNCE_MS)
            {
                g_tilt_kill = 1;
                g_tilt_unkill_since = 0;
                HAL_GPIO_WritePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin, 1);
                PWM_SetSafe();
                g_ctrl_mode = CTRL_MANUAL;
            }
        }
        else g_tilt_kill_since = 0;
    }
    else
    {
        if (aroll < TILT_UNKILL_DEG && apitch < TILT_UNKILL_DEG)
        {
            if (g_tilt_unkill_since == 0) g_tilt_unkill_since = now;
            if ((now - g_tilt_unkill_since) >= TILT_UNKILL_HOLD_MS)
            {
                g_tilt_kill = 0;
                g_tilt_kill_since = 0;
                HAL_GPIO_WritePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin, 0);
            }
        }
        else g_tilt_unkill_since = 0;
    }

    g_imu_new = 1;
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

    if (g_tilt_kill) { PWM_SetSafe(); return; }
    if (!g_att_valid) return;

    if (g_target_base_pwm > PWM_MAX) g_target_base_pwm = PWM_MAX;

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

    if (!g_i_enabled)
    {
        if (g_stab_base_pwm >= I_ENABLE_BASE_PWM) g_i_enabled = 1;
    }
    else
    {
        if (g_stab_base_pwm <= I_DISABLE_BASE_PWM) g_i_enabled = 0;
    }

    float roll_meas_raw, pitch_meas_raw;
    float roll_rate_raw, pitch_rate_raw;

#if IMU_SWAP_ROLL_PITCH
    roll_meas_raw  = (float)g_pitch_deg - (float)g_level_pitch_off;
    pitch_meas_raw = (float)g_roll_deg  - (float)g_level_roll_off;

    roll_rate_raw  = (float)g_gyro_pitch_dps;
    pitch_rate_raw = (float)g_gyro_roll_dps;
#else
    roll_meas_raw  = (float)g_roll_deg  - (float)g_level_roll_off;
    pitch_meas_raw = (float)g_pitch_deg - (float)g_level_pitch_off;

    roll_rate_raw  = (float)g_gyro_roll_dps;
    pitch_rate_raw = (float)g_gyro_pitch_dps;
#endif

    float roll_meas  = roll_meas_raw  * STAB_ROLL_SIGN;
    float pitch_meas = pitch_meas_raw * STAB_PITCH_SIGN;

    float roll_rate  = roll_rate_raw  * STAB_ROLL_SIGN;
    float pitch_rate = pitch_rate_raw * STAB_PITCH_SIGN;

    float cmd_roll  = g_cmd_roll_deg  + ROLL_TRIM_DEG;
    float cmd_pitch = g_cmd_pitch_deg + PITCH_TRIM_DEG;

    float err_roll  = cmd_roll  - roll_meas;
    float err_pitch = cmd_pitch - pitch_meas;

    float u_roll_pd  = STAB_KP_ROLL  * err_roll  - STAB_KD_ROLL  * roll_rate;
    float u_pitch_pd = STAB_KP_PITCH * err_pitch - STAB_KD_PITCH * pitch_rate;

    u_roll_pd  = clamp_f(u_roll_pd,  -STAB_U_MAX, STAB_U_MAX);
    u_pitch_pd = clamp_f(u_pitch_pd, -STAB_U_MAX, STAB_U_MAX);

    float yaw_rate = (float)g_gyro_yaw_dps * STAB_YAW_SIGN;
    float yaw_cmd  = (float)g_cmd_yaw_rate_dps * STAB_YAW_SIGN;
    float yaw_rate_err = yaw_cmd - yaw_rate;

    float u_yaw = STAB_KP_YAW_RATE * yaw_rate_err
                + STAB_KI_YAW_RATE * i_yaw
                + STAB_KFF_YAW * yaw_cmd;
    u_yaw = clamp_f(u_yaw, -STAB_YAW_MAX, STAB_YAW_MAX);

    float u_yaw_total = u_yaw + YAW_TRIM;
    u_yaw_total = clamp_f(u_yaw_total, -STAB_YAW_MAX, STAB_YAW_MAX);

    float u_roll  = u_roll_pd  + STAB_KI_ROLL  * i_roll;
    float u_pitch = u_pitch_pd + STAB_KI_PITCH * i_pitch;

    u_roll  = clamp_f(u_roll,  -STAB_U_MAX, STAB_U_MAX);
    u_pitch = clamp_f(u_pitch, -STAB_U_MAX, STAB_U_MAX);

    int32_t base = (int32_t)g_stab_base_pwm;

    float c_lf, c_rf, c_lb, c_rb;

#if YAW_MIX_FRONT_CCW_REAR_CW
    c_lf = (u_pitch + u_roll + u_yaw_total);
    c_rf = (u_pitch - u_roll + u_yaw_total);
    c_lb = (-u_pitch + u_roll - u_yaw_total);
    c_rb = (-u_pitch - u_roll - u_yaw_total);
#else
    c_lf = (u_pitch + u_roll - u_yaw_total);
    c_rf = (u_pitch - u_roll + u_yaw_total);
    c_lb = (-u_pitch + u_roll + u_yaw_total);
    c_rb = (-u_pitch - u_roll - u_yaw_total);
#endif

    int32_t nav = g_nav_bias;
    switch ((uint8_t)g_last_nav_action)
    {
        case NAV_FORWARD: c_lb += (float)nav; c_rb += (float)nav; c_lf -= (float)nav; c_rf -= (float)nav; break;
        case NAV_BACK:    c_lb -= (float)nav; c_rb -= (float)nav; c_lf += (float)nav; c_rf += (float)nav; break;
        case NAV_LEFT:    c_rb += (float)nav; c_rf += (float)nav; c_lb -= (float)nav; c_lf -= (float)nav; break;
        case NAV_RIGHT:   c_lb += (float)nav; c_lf += (float)nav; c_rb -= (float)nav; c_rf -= (float)nav; break;
        default: break;
    }

    float cgk = CgBiasScale(g_stab_base_pwm);
    float pitch_bias = (float)CG_PITCH_BIAS_PWM * cgk;
    float roll_bias  = (float)CG_ROLL_BIAS_PWM  * cgk;

    c_lf += pitch_bias;
    c_rf += pitch_bias;
    c_lb -= pitch_bias;
    c_rb -= pitch_bias;

    c_lf += roll_bias;
    c_lb += roll_bias;
    c_rf -= roll_bias;
    c_rb -= roll_bias;

    float k = Mixer_ComputeScale(base, c_lf, c_rf, c_lb, c_rb);
    int sat = (k < 0.999f) ? 1 : 0;

    if (!sat && g_i_enabled)
    {
        i_roll  += err_roll  * dt;
        i_pitch += err_pitch * dt;
        i_yaw   += yaw_rate_err * dt;

        i_roll  = clamp_f(i_roll,  -STAB_I_MAX, STAB_I_MAX);
        i_pitch = clamp_f(i_pitch, -STAB_I_MAX, STAB_I_MAX);
        i_yaw   = clamp_f(i_yaw,   -STAB_YAW_I_MAX, STAB_YAW_I_MAX);
    }
    else
    {
        if (!g_i_enabled)
        {
            i_roll  *= (1.0f - 0.8f * dt);
            i_pitch *= (1.0f - 0.8f * dt);
            i_yaw   *= (1.0f - 0.8f * dt);
        }
    }

    float out_lf_f = ((float)base + k * c_lf) * MOTOR_GAIN_LF + MOTOR_OFFS_LF;
    float out_rf_f = ((float)base + k * c_rf) * MOTOR_GAIN_RF + MOTOR_OFFS_RF;
    float out_lb_f = ((float)base + k * c_lb) * MOTOR_GAIN_LB + MOTOR_OFFS_LB;
    float out_rb_f = ((float)base + k * c_rb) * MOTOR_GAIN_RB + MOTOR_OFFS_RB;

    int32_t lf = (int32_t)lroundf(out_lf_f);
    int32_t rf = (int32_t)lroundf(out_rf_f);
    int32_t lb = (int32_t)lroundf(out_lb_f);
    int32_t rb = (int32_t)lroundf(out_rb_f);

    PwmPayload_t out;
    out.motor_lf = clamp_u16(lf, PWM_STAB_FLOOR, PWM_MAX);
    out.motor_rf = clamp_u16(rf, PWM_STAB_FLOOR, PWM_MAX);
    out.motor_lb = clamp_u16(lb, PWM_STAB_FLOOR, PWM_MAX);
    out.motor_rb = clamp_u16(rb, PWM_STAB_FLOOR, PWM_MAX);

    SetPwm(&out);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();

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

  while (1)
  {
    IMU_UpdateContinuous();

    if (!g_armed || g_tilt_kill)
      PWM_SetSafe();
    else if (g_ctrl_mode == CTRL_STAB)
    {
      if (g_imu_new)
      {
        g_imu_new = 0;
        ControlStep_Stabilize();
      }
    }

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

            i_roll = 0.0f;
            i_pitch = 0.0f;
            i_yaw = 0.0f;
            g_i_enabled = 0;

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

            g_target_cmd_roll_deg  = 0.0f;
            g_target_cmd_pitch_deg = 0.0f;
            g_target_cmd_yaw_rate_dps = 0.0f;

            g_last_nav_action = NAV_STOP;
            g_nav_bias = 0;

            i_roll = 0.0f;
            i_pitch = 0.0f;
            i_yaw = 0.0f;
            g_i_enabled = 0;

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

          i_roll = 0.0f;
          i_pitch = 0.0f;
          i_yaw = 0.0f;
          g_i_enabled = 0;

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

            g_last_nav_action = np.action;
            g_nav_bias = 0;

            i_roll = 0.0f;
            i_pitch = 0.0f;
            i_yaw = 0.0f;
            g_i_enabled = 0;

            g_target_cmd_roll_deg  = 0.0f;
            g_target_cmd_pitch_deg = 0.0f;
            g_target_cmd_yaw_rate_dps = 0.0f;

            switch ((uint8_t)np.action)
            {
              case NAV_STOP: break;
              case NAV_FORWARD:   g_target_cmd_pitch_deg = -NAV_TILT_DEG; break;
              case NAV_BACK:      g_target_cmd_pitch_deg = +NAV_TILT_DEG; break;
              case NAV_LEFT:      g_target_cmd_roll_deg  = -NAV_TILT_DEG; break;
              case NAV_RIGHT:     g_target_cmd_roll_deg  = +NAV_TILT_DEG; break;
              case NAV_YAW_LEFT:  g_target_cmd_yaw_rate_dps = -NAV_YAW_RATE_DPS; break;
              case NAV_YAW_RIGHT: g_target_cmd_yaw_rate_dps = +NAV_YAW_RATE_DPS; break;
              default: break;
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
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
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

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
