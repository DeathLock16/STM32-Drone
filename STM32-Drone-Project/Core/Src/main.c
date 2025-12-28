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
  memset(&imu, 0, sizeof(imu));
  memset(&filter, 0, sizeof(filter));
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

  printf("Boot\r\n");

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

  /* jawna inicjalizacja quaterniona */
  filter.q0 = 1.0f;
  filter.q1 = 0.0f;
  filter.q2 = 0.0f;
  filter.q3 = 0.0f;

  HAL_Delay(100);
  lastTick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    MPU6050_ReadRaw(&hi2c1, &imu);
    MPU6050_ComputeScaled(&imu);

    float acc_norm = imu.acc_x * imu.acc_x +
                     imu.acc_y * imu.acc_y +
                     imu.acc_z * imu.acc_z;

    if (acc_norm < 0.5f || acc_norm > 1.5f)
    {
        HAL_Delay(1);
        continue;
    }

    uint32_t now = HAL_GetTick();
    float dt = (now - lastTick) / 1000.0f;
    lastTick = now;

    if (dt <= 0.0f || dt > 0.1f)
    {
        HAL_Delay(1);
        continue;
    }

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

    float yaw   = atan2f(
        2.0f * (filter.q0 * filter.q3 + filter.q1 * filter.q2),
        1.0f - 2.0f * (filter.q2 * filter.q2 + filter.q3 * filter.q3)
    ) * RAD2DEG;

    printf("{\"roll\":%d,\"pitch\":%d,\"yaw\":%d}\r\n",
           (int)(roll * 100),
           (int)(pitch * 100),
           (int)(yaw * 100)
    );


    HAL_Delay(10); /* ~100 Hz */

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
