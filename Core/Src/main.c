/*
 * Authors: Aleksandr Safronov, Quoc Trung Trun
 * File: main.c
 * Description:
 *   This file contains the core implementation of the motion tracking camera mount,
 *   including algorithms for infrared/thermal tracking and servo motor control.
 *
 *   Hardware:
 *     - Microcontroller: STM32F446RE (64-pin configuration).
 *     - Peripherals: Infrared/thermal sensors, servo motors (controlled via UART).
 *
 *   Software Dependencies:
 *     - MLX90640 API (C++): Used for processing thermal data from the infrared sensors.
 *     - SCSLib: A library for controlling servo motors via UART.
 *
 * License:
 *   Licensed under CC BY-NC 4.0. See LICENSE file for details.
 *   Copyright © 2024 Aleksandr Safronov, Quoc Trung Trun
 *
 * Date:
 *   Last Modified: 2024-12-05
 *
 * Notes:
 *   - The motion tracking algorithm is optimized for medium-distance, indoor use.
 *   - Ensure all sensors are calibrated before running the tracking system.
 */

#include "main.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "SCServo.h"
#include "uart.h"
#include <string.h>

UART_HandleTypeDef huart2;
extern void setup(void);
extern void loop(void);
extern void rotateConstantSpeed(uint8_t);

static void MX_USART2_UART_Init(void);

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//  * Configure the main internal regulator output voltage

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//  * Initializes the CPU, AHB and APB busses clocks

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;//8M�ⲿ����
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
//  * Initializes the CPU, AHB and APB busses clocks

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void initSys(void)
{
	HAL_Init();
	SystemClock_Config();
	 // Enable clocks before initializing peripherals
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE(); // For servo communication
	__HAL_RCC_USART2_CLK_ENABLE(); // For console output

	MX_USART2_UART_Init(); // Initialize UART2 for console output
}



int main()
{
	initSys();
	setup();
	while(1){
//		char testMsg[] = "UART Test Message
";
//		HAL_UART_Transmit(&huart2, (uint8_t*)testMsg, strlen(testMsg), HAL_MAX_DELAY);
		loop();

		WheelMode(1);
		rotateConstantSpeed(1);
		//WritePosEx(1, 4095, 2250, 30); // Servo motor (ID1), with maximum speed V=2250 steps/sec, acceleration A=50 (50*100 steps/sec²), moves to position P1=4095
		//HAL_Delay(2270); // Delay calculation: [(P1-P0)/V]*1000 + [V/(A*100)]*1000

	  	//WritePosEx(1, 0, 2250, 30); // Servo motor (ID1), with maximum speed V=2250 steps/sec, acceleration A=50 (50*100 steps/sec²), moves to position P1=0
	  	//HAL_Delay(2270); // Delay calculation: [(P1-P0)/V]*1000 + [V/(A*100)]*1000


	}
}

static void MX_USART2_UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

/*	 *USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10    ------> USART1_RX
*/

	    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {

  }

}
