/*
 * Authors: Aleksandr Safronov, Quoc Trung Trun
 * File: i2c.c
 * Description:
 *   This file provides I2C configuration and initialization routines for 
 *   STM32 devices, originally generated using STM32CubeMX and modified 
 *   for future use with the MLX90640 thermal sensor.
 *
 *   Modifications:
 *     - Adjustment of GPIO configurations for I2C instance selection.
 *     - Initialization and deinitialization routines to allow dynamic reconfiguration.
 *
 * License:
 *   Licensed under the BSD 3-Clause License (© 2024 STMicroelectronics).
 *   Modifications © 2024 Aleksandr Safronov, Quoc Trung Trun, licensed under
 *   the same terms.
 */
/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;


/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}


/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}


/* I2C3 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

}
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
    {
    /* USER CODE BEGIN I2C2_MspInit 0 */

    /* USER CODE END I2C2_MspInit 0 */

      __HAL_RCC_GPIOB_CLK_ENABLE();

      //PB8 --- SDA PB9 --- SCL FOR I2C1

      GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* I2C2 clock enable */
      __HAL_RCC_I2C1_CLK_ENABLE();
    /* USER CODE BEGIN I2C2_MspInit 1 */

    /* USER CODE END I2C2_MspInit 1 */
    }
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /*
    PB3 --- SDA PB10 --- SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
  if(i2cHandle->Instance==I2C3)
    {
    /* USER CODE BEGIN I2C2_MspInit 0 */

    /* USER CODE END I2C2_MspInit 0 */

      __HAL_RCC_GPIOB_CLK_ENABLE();

      //PA8 --- SDA PC9 --- SCL FOR I2C3

      GPIO_InitStruct.Pin = GPIO_PIN_8;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


      GPIO_InitStruct.Pin = GPIO_PIN_9;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
      /* I2C2 clock enable */
      __HAL_RCC_I2C3_CLK_ENABLE();
    /* USER CODE BEGIN I2C3_MspInit 1 */

    /* USER CODE END I2C3_MspInit 1 */
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
  if(i2cHandle->Instance==I2C1)
  {
	  /* USER CODE BEGIN I2C2_MspDeInit 0 */

	  /* USER CODE END I2C2_MspDeInit 0 */
	    /* Peripheral clock disable */
	    __HAL_RCC_I2C1_CLK_DISABLE();

	    //PB8 --- SDA PB9 --- SCL FOR I2C1

	    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);


	  /* USER CODE BEGIN I2C2_MspDeInit 1 */

	  /* USER CODE END I2C2_MspDeInit 1 */
	  }

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();
  
    //PB3 --- SDA PB10 --- SCL FOR I2C2

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_10);


  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }

  if(i2cHandle->Instance==I2C3)
    {
    /* USER CODE BEGIN I2C2_MspDeInit 0 */

    /* USER CODE END I2C2_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_I2C3_CLK_DISABLE();

      //PA8 --- SDA PC9 --- SCL FOR I2C3

      // De-initialize GPIO pin PA8 (SDA)
      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

      // De-initialize GPIO pin PC9 (SCL)
      HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
    }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
