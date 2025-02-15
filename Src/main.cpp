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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rabcl/utils/type.hpp"
#include "rabcl/interface/uart.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t control_count = 0;
uint16_t can_count = 0;
char printf_buf[100];

rabcl::Info robot_data;
rabcl::Uart* uart;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim15)
  {
    control_count++;
    if (control_count >= 10) // 100Hz
    {
      control_count = 0;
    }

    // ---can
    can_count++;
    if (can_count >= 25) // 40Hz
    {
      can_count = 0;

      // check robot data
      // snprintf(printf_buf, 100, "chassis_vel_x: %f\n", robot_data.chassis_vel_x_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      // snprintf(printf_buf, 100, "chassis_vel_y: %f\n", robot_data.chassis_vel_y_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      // snprintf(printf_buf, 100, "chassis_vel_z: %f\n", robot_data.chassis_vel_z_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      // snprintf(printf_buf, 100, "yaw_vel: %f\n", robot_data.yaw_vel_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      // snprintf(printf_buf, 100, "pitch_vel_: %f\n", robot_data.pitch_vel_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      // snprintf(printf_buf, 100, "load_mode: %d\n", robot_data.load_mode_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      // snprintf(printf_buf, 100, "fire_mode: %d\n", robot_data.fire_mode_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      // snprintf(printf_buf, 100, "speed_mode: %d\n", robot_data.speed_mode_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      // snprintf(printf_buf, 100, "chassis_mode: %d\n", robot_data.chassis_mode_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  if (uart->UpdateData(robot_data))
  {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  }
  HAL_UART_Receive_IT(&huart2, uart->uart_receive_buffer_, 8);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uart = new rabcl::Uart();

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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim15);
  HAL_CAN_Start(&hcan);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_UART_Receive_IT(&huart2, uart->uart_receive_buffer_, 8);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
