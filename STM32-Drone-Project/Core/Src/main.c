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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t imu;
Madgwick_t filter;

uint32_t lastTick;

static ImuPayload_t g_imu_last;
static uint8_t g_imu_ready = 0;
static uint32_t g_imu_last_update = 0;

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

        //HAL_GPIO_TogglePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin);
        HAL_Delay(500);
        //HAL_GPIO_TogglePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin);
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
       // HAL_GPIO_WritePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin, GPIO_PIN_SET);
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
	      //printf("MPU6050 init error\r\n");
	      Error_Handler();
	  }

	  uint8_t who = 0;
	  if (MPU6050_ReadWhoAmI(&hi2c1, &who) == HAL_OK)
	  {
	      //printf("MPU WHO_AM_I = 0x%02X\r\n", who);
	  }

	  //printf("Calibrating gyro...\r\n");
	  if (MPU6050_CalibrateGyro(&hi2c1, &imu, 1000) != HAL_OK)
	  {
	      //printf("Gyro calibration failed\r\n");
	      Error_Handler();
	  }

	  Madgwick_Init(&filter, 0.08f);

	  /* jawna inicjalizacja quaterniona */
	  filter.q0 = 1.0f;
	  filter.q1 = 0.0f;
	  filter.q2 = 0.0f;
	  filter.q3 = 0.0f;
}

static void IMU_UpdateContinuous(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - g_imu_last_update) < 10)  // 100 Hz
        return;

    g_imu_last_update = now;

    MPU6050_ReadRaw(&hi2c1, &imu);
    MPU6050_ComputeScaled(&imu);

    float acc_norm =
        imu.acc_x * imu.acc_x +
        imu.acc_y * imu.acc_y +
        imu.acc_z * imu.acc_z;

    if (acc_norm < 0.5f || acc_norm > 1.5f)
        return;

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

    Madgwick_UpdateIMU(
        &filter,
        imu.gyro_x * DEG2RAD,
        imu.gyro_y * DEG2RAD,
        imu.gyro_z * DEG2RAD,
        imu.acc_x,
        imu.acc_y,
        imu.acc_z,
        dt
    );

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

    g_imu_ready = 1;
}


ImuResult_t IMU_ParseData(ImuAngles_t *out)
{
    if (!out)
        return IMU_ERROR;

    MPU6050_ReadRaw(&hi2c1, &imu);
    MPU6050_ComputeScaled(&imu);

    float acc_norm =
        imu.acc_x * imu.acc_x +
        imu.acc_y * imu.acc_y +
        imu.acc_z * imu.acc_z;

    if (acc_norm < 0.5f || acc_norm > 1.5f)
        return IMU_SKIP;

    uint32_t now = HAL_GetTick();

    if (lastTick == 0)
    {
        lastTick = now;
        return IMU_SKIP;
    }

    float dt = (now - lastTick) * 0.001f;
    lastTick = now;

    if (dt <= 0.0f)
        return IMU_SKIP;

    if (dt > 0.02f)
        dt = 0.02f;

    Madgwick_UpdateIMU(
        &filter,
        imu.gyro_x * DEG2RAD,
        imu.gyro_y * DEG2RAD,
        imu.gyro_z * DEG2RAD,
        imu.acc_x,
        imu.acc_y,
        imu.acc_z,
        dt
    );

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

    out->roll  = (int16_t)(roll  * 100.0f);
    out->pitch = (int16_t)(pitch * 100.0f);
    out->yaw   = (int16_t)(yaw   * 100.0f);

    return IMU_OK;
}

static void PWM_StartAll(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim1);
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
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p->motor_lb);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p->motor_lf);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p->motor_rf);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, p->motor_rb);
}

static void PWM_SetSafe(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  PWM_StartAll();
  PWM_SetSafe();

  ESP_Init();

  IMU_Init();
  lastTick = HAL_GetTick();
  g_imu_last_update = lastTick;

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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	      IMU_UpdateContinuous();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
