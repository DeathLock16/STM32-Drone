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
#include "gpio.h"
#include "i2c.h"
#include "usart.h"

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

static void ESP_LeaveSendMode(void) {
	const char* cmd = "+";
	HAL_UART_Transmit(&huart6, cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6, cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6, cmd, strlen(cmd), HAL_MAX_DELAY);
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
            break; // sukces
        }

        HAL_GPIO_TogglePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin);
        HAL_Delay(500);
        HAL_GPIO_TogglePin(LED_SIGNAL_GPIO_Port, LED_SIGNAL_Pin);
        HAL_Delay(500);// 1 sekunda przerwy
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
    ESP_Send("AT+CIPSEND", 200);
    uint8_t dump;
    while (HAL_UART_Receive(&huart6, &dump, 1, 10) == HAL_OK)
    {
        // discard
    }

    // NO CIPMODE
    // NO CIPSEND here
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
  //memset(&imu, 0, sizeof(imu));
  //memset(&filter, 0, sizeof(filter));
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */

  //printf("Boot\r\n");
  ESP_Init();
/*
  if (MPU6050_Init(&hi2c1) != HAL_OK)
  {
      printf("MPU6050 init error\r\n");
      Error_Handler();
  }

  uint8_t who = 0;
  if (MPU6050_ReadWhoAmI(&hi2c1, &who) == HAL_OK)
  {
      printf("MPU WHO_AM_I = 0x%02X\r\n", who);
  }

  printf("Calibrating gyro...\r\n");
  if (MPU6050_CalibrateGyro(&hi2c1, &imu, 1000) != HAL_OK)
  {
      printf("Gyro calibration failed\r\n");
      Error_Handler();
  }
  printf("Gyro bias: %.1f %.1f %.1f\r\n",
         imu.gyro_bias_x,
         imu.gyro_bias_y,
         imu.gyro_bias_z);

  Madgwick_Init(&filter, 0.08f);

  filter.q0 = 1.0f;
  filter.q1 = 0.0f;
  filter.q2 = 0.0f;
  filter.q3 = 0.0f;

  HAL_Delay(100);
  lastTick = HAL_GetTick();
  */
  Protocol_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    uint8_t b;

	    if (HAL_UART_Receive(&huart6, &b, 1, HAL_MAX_DELAY) == HAL_OK)
	    {
	        if (Protocol_PushByte(b))
	        {
	            const ProtoFrame_t *rx = Protocol_GetFrame();
	            ProtoFrame_t tx;

	            if (Protocol_BuildResponse(rx, &tx))
	            {
	                HAL_UART_Transmit(&huart6,
	                                  (uint8_t*)&tx,
	                                  sizeof(tx),
	                                  HAL_MAX_DELAY);
	            }
	        }
	    }


    /* USER CODE END 3 */
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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
