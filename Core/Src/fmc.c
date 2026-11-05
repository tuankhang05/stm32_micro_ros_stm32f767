/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : FMC.c
  * Description        : This file provides code for the configuration
  *                      of the FMC peripheral.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fmc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SRAM_HandleTypeDef hsram1;

/* FMC initialization function */
void MX_FMC_Init(void)
{
  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  /* Toàn bộ cấu hình FMC phục vụ LCD ILI9341 đã được loại bỏ.
   * Không khởi tạo FMC để giải phóng hoàn toàn các chân GPIO,
   * đặc biệt là PD14 (FMC_D0 cũ) hiện dùng làm ngắt EXTI14 cho tín hiệu Hall HC_R.
   */

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}
