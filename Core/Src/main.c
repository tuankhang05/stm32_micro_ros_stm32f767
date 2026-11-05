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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot_params.h"
#include "encoder.h"
#include "motor.h"
#include "encoder_speed.h"
#include "pid.h"
#include "odometry.h"
#include "hall_sensor.h"
#include "ps2xlib.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define min(a,b) (((a)<(b))?(a):(b))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//ETH_HandleTypeDef heth;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //SCB_DisableDCache();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  //MX_FMC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  /* Khởi tạo hệ thống Hall Sensor, Encoder wrapper và Motor */
  Hall_Init();
  Hall_Reset(HALL_MOTOR_LEFT);
  Hall_Reset(HALL_MOTOR_RIGHT);
  Encoder_Init();
  EncoderSpeed_Init();
  Motor_Init();

  //int32_t PID_sampletime = 5;

  PID(&PosPID1, &CurPos1, &PosPIDOut1, &DesiredPos1, Kp1, Ki1, Kd1, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&PosPID1, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&PosPID1, MOTORCONTROL_PERIOD_MS);
  PID_SetOutputLimits(&PosPID1, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  PID(&PosPID2, &CurPos2, &PosPIDOut2, &DesiredPos2, Kp2, Ki2, Kd2, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&PosPID2, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&PosPID2, MOTORCONTROL_PERIOD_MS);
  PID_SetOutputLimits(&PosPID2, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  PID(&SpeedPID1, &CurSpeed1, &SpeedPIDOut1, &DesiredSpeed1, Kp1, Ki1, Kd1, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&SpeedPID1, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&SpeedPID1, MOTORCONTROL_PERIOD_MS);
  PID_SetOutputLimits(&SpeedPID1, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  PID(&SpeedPID2, &CurSpeed2, &SpeedPIDOut2, &DesiredSpeed2, Kp2, Ki2, Kd2, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&SpeedPID2, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&SpeedPID2, MOTORCONTROL_PERIOD_MS);
  PID_SetOutputLimits(&SpeedPID2, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  PID_Init(&PosPID1); PID_Init(&PosPID2);
  PID_Init(&SpeedPID1); PID_Init(&SpeedPID2);

//  const char test_msg[] = "USART2 VCP OK\r\n";
//  while (1)
//  {
//	  HAL_StatusTypeDef st = HAL_UART_Transmit(&huart2,
//	                                           (uint8_t*)test_msg,
//	                                           sizeof(test_msg)-1,
//	                                           10);
//
//    HAL_Delay(500);   // gửi mỗi 500 ms
//  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  *   STM32F767 at 216 MHz requires VOS Scale 1 + Over-Drive
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  * HSE = 8 MHz (NUCLEO-F767ZI onboard oscillator)
  * PLL_M=4, PLL_N=216, PLL_P=2, PLL_Q=9 => SYSCLK=216 MHz, USB/SDIO=48 MHz
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;   /* NUCLEO uses MCO from ST-Link (bypass) */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;   /* SYSCLK = 216 MHz */
  RCC_OscInitStruct.PLL.PLLQ = 9;               /* USB/SDIO = 48 MHz */
  RCC_OscInitStruct.PLL.PLLR = 2;               /* PLLR (STM32F767 extra output) */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode to reach 216 MHz
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  *   HCLK=216 MHz, APB1=54 MHz (max 54), APB2=108 MHz (max 108)
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  // -------- Region0: default deny (giữ giống bạn) --------
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = 0x00000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87; // như bạn
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}



/* USER CODE END 4 */



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
