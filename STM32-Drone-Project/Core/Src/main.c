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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG2RAD 0.0174532925f
#define RAD2DEG 57.2957795f

#define IMU_UPDATE_PERIOD_MS 10u   /* 100 Hz */

#define ACC_LPF_CUTOFF_HZ  20.0f   /* 15–25 Hz: startowo 20 Hz */
#define ACC_G_MIN          0.80f
#define ACC_G_MAX          1.20f

#define TILT_KILL_DEG      35.0f
#define TILT_UNKILL_DEG    25.0f

#define TILT_KILL_DEBOUNCE_MS   50u   /* musi trwać >45° przez 50 ms */
#define TILT_UNKILL_HOLD_MS    300u   /* musi być <35° przez 300 ms żeby odblokować */

#define PWM_MIN 0
#define PWM_MAX 10000

#define STAB_KP_ROLL 25.0f
#define STAB_KD_ROLL 2.0f
#define STAB_KP_PITCH 25.0f
#define STAB_KD_PITCH 2.0f

#define STAB_KP_YAW_RATE 2.0f
#define STAB_YAW_SIGN 1.0f
#define STAB_YAW_MAX 1200.0f

#define STAB_ROLL_SIGN 1.0f
#define STAB_PITCH_SIGN 1.0f
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

/* track last pwm command (for debug / potential safety extensions) */
static PwmPayload_t g_last_pwm_cmd;
static uint8_t g_have_pwm_cmd = 0;

typedef enum {
	CTRL_MANUAL,
	CTRL_STAB
} ControlMode_t;

static volatile ControlMode_t g_ctrl_mode = CTRL_MANUAL;
static volatile uint16_t g_stab_base_pwm = 0;

static volatile float g_roll_deg = 0.0f;
static volatile float g_pitch_deg = 0.0f;
static volatile float g_gyro_roll_dps = 0.0f;
static volatile float g_gyro_pitch_dps = 0.0f;
static volatile float g_gyro_yaw_dps = 0.0f;
static volatile uint8_t g_att_valid = 0;

static volatile uint8_t g_armed = 0;
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

/* --- ACC LPF helpers --- */
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

static inline uint16_t clamp_u16(int32_t v, int32_t lo, int32_t hi) {
	if (v < lo) return (uint16_t)lo;
	if (v > hi) return (uint16_t)hi;
	return (uint16_t)v;
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

static uint8_t CalcCrcPayload(uint8_t dir, uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t crc = (uint8_t)(dir ^ cmd ^ len);
    for (uint8_t i = 0; i < len; i++)
        crc ^= payload[i];
    return crc;
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
    {
        Error_Handler();
    }

    uint8_t who = 0;
    (void)MPU6050_ReadWhoAmI(&hi2c1, &who);

    if (MPU6050_CalibrateGyro(&hi2c1, &imu, 1000) != HAL_OK)
    {
        Error_Handler();
    }

    Madgwick_Init(&filter, 0.05f);

    filter.q0 = 1.0f;
    filter.q1 = 0.0f;
    filter.q2 = 0.0f;
    filter.q3 = 0.0f;

    /* reset LPF state */
    ax_f = 0.0f; ay_f = 0.0f; az_f = 1.0f;

    /* reset timers */
    lastTick = HAL_GetTick();
    g_imu_last_update = lastTick;

    g_tilt_kill = 0;
    g_tilt_kill_since = 0;
    g_tilt_unkill_since = 0;

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
    /* always remember last command */
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

/* IMU continuous update @ ~100Hz */
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

    /* LPF on accelerometer */
    lpf3(dt, ACC_LPF_CUTOFF_HZ, imu.acc_x, imu.acc_y, imu.acc_z, &ax_f, &ay_f, &az_f);

    /* ACC validity by |a| */
    int acc_ok = accel_norm_ok(ax_f, ay_f, az_f, ACC_G_MIN, ACC_G_MAX);

    /* Gyro must be rad/s for this Madgwick (we convert deg/s -> rad/s) */
    float gx = imu.gyro_x * DEG2RAD;
    float gy = imu.gyro_y * DEG2RAD;
    float gz = imu.gyro_z * DEG2RAD;

    if (acc_ok)
    {
        Madgwick_UpdateIMU(&filter, gx, gy, gz, ax_f, ay_f, az_f, dt);
    }
    else
    {
        /* gyro-only */
        Madgwick_UpdateIMU(&filter, gx, gy, gz, 0.0f, 0.0f, 0.0f, dt);
    }

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
    g_gyro_yaw_dps = imu.gyro_z;

    g_gyro_roll_dps = imu.gyro_x;
    g_gyro_pitch_dps = imu.gyro_y;

    g_att_valid = 1;

    /* ---- Tilt kill with debounce + hysteresis ---- */
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
        /* already killed: require stable < UNKILL threshold for some time */
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

/* Return last snapshot only (do NOT update filter here) */
ImuResult_t IMU_ParseData(ImuAngles_t *out)
{
    if (!out)
        return IMU_ERROR;

    if (!g_imu_ready)
        return IMU_SKIP;

    out->roll  = g_imu_last.roll;
    out->pitch = g_imu_last.pitch;
    out->yaw   = g_imu_last.yaw;
    return IMU_OK;
}

static void ControlStep_Stabilize(void)
{
    if (g_tilt_kill)
    {
        PWM_SetSafe();
        return;
    }

    if (!g_att_valid)
        return;

    float roll  = (float)g_roll_deg * STAB_ROLL_SIGN;
    float pitch = (float)g_pitch_deg * STAB_PITCH_SIGN;

    float roll_rate  = (float)g_gyro_roll_dps * STAB_ROLL_SIGN;
    float pitch_rate = (float)g_gyro_pitch_dps * STAB_PITCH_SIGN;

    float err_roll  = -roll;
    float err_pitch = -pitch;

    float d_roll  = -roll_rate;
    float d_pitch = -pitch_rate;

    float u_roll  = STAB_KP_ROLL  * err_roll  + STAB_KD_ROLL  * d_roll;
    float u_pitch = STAB_KP_PITCH * err_pitch + STAB_KD_PITCH * d_pitch;

    float yaw_rate = (float)g_gyro_yaw_dps * STAB_YAW_SIGN;
    float u_yaw = STAB_KP_YAW_RATE * yaw_rate;

    if (u_yaw > STAB_YAW_MAX) u_yaw = STAB_YAW_MAX;
    if (u_yaw < -STAB_YAW_MAX) u_yaw = -STAB_YAW_MAX;

    int32_t base = (int32_t)g_stab_base_pwm;

    PwmPayload_t out;

    int32_t lf = base + (int32_t)lroundf(u_pitch + u_roll - u_yaw);
    int32_t rf = base + (int32_t)lroundf(u_pitch - u_roll + u_yaw);
    int32_t lb = base + (int32_t)lroundf(-u_pitch + u_roll + u_yaw);
    int32_t rb = base + (int32_t)lroundf(-u_pitch - u_roll - u_yaw);

    out.motor_lf = clamp_u16(lf, PWM_MIN, PWM_MAX);
    out.motor_rf = clamp_u16(rf, PWM_MIN, PWM_MAX);
    out.motor_lb = clamp_u16(lb, PWM_MIN, PWM_MAX);
    out.motor_rb = clamp_u16(rb, PWM_MIN, PWM_MAX);

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

                      g_stab_base_pwm = sp.base_pwm;
                      g_ctrl_mode = CTRL_STAB;
                      g_armed = 1;

                      tx_len = Protocol_BuildFrame(DIR_STM_TO_PC, CMD_STAB_ACK,
                                                   (const uint8_t*)&sp, STAB_PAYLOAD_SIZE,
                                                   tx, sizeof(tx));
                  }
              }
              else if (rx->cmd == CMD_STAB_OFF)
              {
                  g_ctrl_mode = CTRL_MANUAL;
                  PWM_SetSafe();
                  tx_len = Protocol_BuildFrame(DIR_STM_TO_PC, CMD_STAB_OFF_ACK, NULL, 0, tx, sizeof(tx));
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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
