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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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

        HAL_GPIO_TogglePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin);
        HAL_Delay(500);
        HAL_GPIO_TogglePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin);
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
        HAL_GPIO_WritePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin, GPIO_PIN_SET);
        while (1) { }
    }

    uint8_t dump;
    while (HAL_UART_Receive(&huart6, &dump, 1, 10) == HAL_OK)
    {
    }
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


  Protocol_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint8_t b;

	      if (HAL_UART_Receive(&huart6, &b, 1, HAL_MAX_DELAY) != HAL_OK)
	          continue;

	      ProtoResult_t r = Protocol_PushByte(b);

	      if (r == PROTO_OK)
	      {
	          const ProtoRxFrame_t *rx = Protocol_GetFrame();

	          uint8_t tx[32];
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
	          else
	          {
	              tx_len = Protocol_BuildStatusFrame(ST_ERR_UNKNOWN_CMD, tx, sizeof(tx));
	          }

	          if (tx_len > 0)
	          {
	              HAL_UART_Transmit(&huart6, tx, tx_len, HAL_MAX_DELAY);
	          }
	      }
	      else if (r == PROTO_ERROR)
	      {
	          uint8_t tx[16];
	          uint8_t tx_len = Protocol_BuildStatusFrame(Protocol_GetLastError(), tx, sizeof(tx));

	          if (tx_len > 0)
	          {
	              HAL_UART_Transmit(&huart6, tx, tx_len, HAL_MAX_DELAY);
	          }
	      }
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
