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
#include <string.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
// Servo Motors Includes
#include "stm32f4xx.h"
#include "SCServo.h"
#include "uart.h"

#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33 //default address
#define	 RefreshRate FPS16HZ //max speed 16
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air
#define  TARGET_MIN_TEMP 27
#define  TARGET_MAX_TEMP 37

//calibration for average count thingy
#define NUM_SAMPLES 10
#define NUM_AVERAGES 5
#define NUM_SENSORS 1
#define SENSOR_ARRAY_SIZE 768
#define ROWS 24
#define COLUMNS 32

#define GROUP_SIZE 4//Consider Wiring this up to a potentiometer to adjust accuracy


I2C_HandleTypeDef *hi2c;
UART_HandleTypeDef huart2;
//extern I2C_HandleTypeDef hi2c1;
//volatile I2C_HandleTypeDef hi2c1;
//volatile I2C_HandleTypeDef hi2c3;

// Servo Motor external declarations
extern void setup(void);
extern void loop(void);
extern void rotateConstantSpeed(uint8_t);
extern float rotateToTargetColumn(uint8_t, int);
extern float rotateToTargetRow(uint8_t, int);
static void MX_USART2_UART_Init(void);
float emissivity=0.95;//0.95 is default
int status;
char MLX90640_Test_Buffer[255];
paramsMLX90640 mlx90640; //struct found in GitHub libraries
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t eeMLX90640[832];
    paramsMLX90640 mlx90640;
    uint16_t frame[834];
    float mlx90640To[768];
    int targetDetected;
    int highestRowGroupStart;
    int highestColGroupStart;
    float avg;
    int highestColumnIndex;
} SensorData;

SensorData sensors[3];


//VERY IMPORTANT
//SENSOR_L
//PB3 --- SDA PB10 --- SCL FOR I2C2

//SENSOR_M
//PB8 --- SDA PB9 --- SCL FOR I2C1

//SENSOR_R
//PA8 --- SDA PC9 --- SCL FOR I2C3

// Functions to process MLX90640 data
int retriever(I2C_HandleTypeDef *hi2c, uint16_t *eeMLX90640, paramsMLX90640 *mlx90640, float *mlx90640To, uint16_t *frame, int *targetDetected, int *highestRowGroupStart, int *highestColGroupStart)
{

	//EEPROM DATA CHECK
	status = MLX90640_DumpEE(hi2c,MLX90640_ADDR, eeMLX90640);
	sprintf(MLX90640_Test_Buffer, "Value of status for debug: %d
", status);
	HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

	//PARAMETER CHECK
	status = MLX90640_ExtractParameters(eeMLX90640, mlx90640);
	sprintf(MLX90640_Test_Buffer, "
New value of status for debug: %d
", status);

    // Get frame data using the correct I2C handle and address
    int status = MLX90640_GetFrameData(hi2c, MLX90640_ADDR, frame);
    if (status < 0)
    {
        sprintf(MLX90640_Test_Buffer, "GetFrame Error: %d
", status);
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
        return -1; // Early exit on error
    }
    // Adjusted function calls to pass mlx90640 without dereferencing
    float vdd = MLX90640_GetVdd(frame, mlx90640); // Pass mlx90640 directly
    float Ta = MLX90640_GetTa(frame, mlx90640);   // Pass mlx90640 directly
    float tr = Ta - TA_SHIFT; // Reflected temperature based on the sensor ambient temperature

    // tr = Ambient Reflected Temperature
    sprintf(MLX90640_Test_Buffer, "vdd:  %f Tr: %f
", vdd, tr);
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    MLX90640_CalculateTo(frame, mlx90640, emissivity, tr, mlx90640To); // Pass mlx90640 directly
    return 0;
}

int overallAvg(float mlx90640To_2[SENSOR_ARRAY_SIZE], int* highestRowGroupStart, int* highestColGroupStart)
{
	float sum = 0.0;
	float overallAverage;

	//TO CALCULATE OVERALL AVERAGE
	for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
	{
	    sum += mlx90640To_2[i];
	    overallAverage = sum / SENSOR_ARRAY_SIZE;
	}
	sprintf(MLX90640_Test_Buffer, "[1;31mOverall Average: %.2f[0m
", overallAverage);
	HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
	return overallAverage;
}

int avgBySector(float mlx90640To_2[SENSOR_ARRAY_SIZE], int* highestRowGroupStart, int* highestColGroupStart)
{
	//GROUP_SIZE IS DEFINED ABOVE ON TOP OF CODE
	//CURRENT GROUP_SIZE IS SET TO 1, CAN BE INCREASE BY THE POWER OF 2(i.e 2,4,8,16) TO
	//INCREASE OR DECREASE ACCURAY OF THE CALCULATIONS

	float highestRowGroupAverage = -1.0;
	float highestColGroupAverage = -1.0;

	*highestRowGroupStart = -1;
	*highestColGroupStart = -1;

    float groupColAverage;
    float groupRowAverage;

    //TO CALCULATE GROUPS OF ROWS AVERAGE
	for (int r = 0; r < ROWS; r += GROUP_SIZE)
	{
	    float groupRowSum = 0.0;
	    int rowsToSum = (r + GROUP_SIZE <= ROWS) ? GROUP_SIZE : (ROWS - r);
	    for (int i = 0; i < rowsToSum; i++)
	    {
	         for (int c = 0; c < COLUMNS; c++)
	         {
	              groupRowSum += mlx90640To_2[(r + i) * COLUMNS + c];
	         }
	    }
	    groupRowAverage = groupRowSum / (rowsToSum * COLUMNS);

	    sprintf(MLX90640_Test_Buffer, "Row Group starting at Row %d Average: %.2f
", r + 1, groupRowAverage);
	    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
	    // Check if this row group has the highest average
	    if (groupRowAverage > highestRowGroupAverage)
	    {
	      	highestRowGroupAverage = groupRowAverage;
	       	*highestRowGroupStart = r + 1;
	    }
	}
	//TO CALCULATE GROUPS OF COLUMNS AVERAGE
	for (int c = 0; c < COLUMNS; c += GROUP_SIZE)
	{
	    float groupColSum = 0.0;
	    int colsToSum = (c + GROUP_SIZE <= COLUMNS) ? GROUP_SIZE : (COLUMNS - c);
	    for (int i = 0; i < colsToSum; i++)
	    {
	        for (int r = 0; r < ROWS; r++)
	        {
	            groupColSum += mlx90640To_2[r * COLUMNS + (c + i)];
	        }
	    }
	    groupColAverage = groupColSum / (colsToSum * ROWS);
	    sprintf(MLX90640_Test_Buffer, "Column Group starting at Column %d Average: %.2f
", c + 1, groupColAverage);
	    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
	    // Check if this column group has the highest average
	    if (groupColAverage > highestColGroupAverage)
	    {
	    	highestColGroupAverage = groupColAverage;
	        *highestColGroupStart = c + 1;
	    }
	}
	//print HIGHEST ROW and COL averages and Overall Average

	if (*highestRowGroupStart != -1)
	{
		sprintf(MLX90640_Test_Buffer, "[1;31mHighest Row Group Average: Row Group starting at Row %d with Average: %.2f[0m
",*highestRowGroupStart, highestRowGroupAverage);
	    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
	}
	if (*highestColGroupStart != -1)
	{
	  	sprintf(MLX90640_Test_Buffer, "[1;31mHighest Column Group Average: Column Group starting at Column %d with Average: %.2f[0m
",*highestColGroupStart, highestColGroupAverage);
	   	HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
	}

	// Alex modified for working with Highest Avg Column - OK LMAO
	return *highestColGroupStart;
}

void setLEDState(int targetDetected) // Example: PA5 used for LED
{

	if (targetDetected == 1)
		{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
	else
		{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
}

int printer(I2C_HandleTypeDef *hi2c, uint16_t *eeMLX90640, paramsMLX90640 *mlx90640, float *mlx90640To,
            uint16_t *frame, int *targetDetected, int *highestRowGroupStart, int *highestColGroupStart)
{
    /* Waveshare data */
    sprintf(MLX90640_Test_Buffer, "
==========================Waveshare==========================
");
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    // FOR LOOP FOR PRINTING OUT VALUES STARTS HERE
    // Print out Column numbers in Blue
    sprintf(MLX90640_Test_Buffer, "     Col"); // Initial spacing for row labels
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    for (int col = 0; col < 32; col++)
    {
        if (col >= (*highestColGroupStart - 1) && col < (*highestColGroupStart - 1 + GROUP_SIZE))
        {
            // Print in white for highest column group
            sprintf(MLX90640_Test_Buffer, " [1;37m%2d [0m  ", col + 1);
        }
        else
        {
            // Print in blue for other columns
            sprintf(MLX90640_Test_Buffer, " [1;34m%2d [0m  ", col + 1);
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
    }
    sprintf(MLX90640_Test_Buffer, "
");
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    for (int i = 0; i < 768; i++) // For loop to print temperature variables
    {
        if (i % 32 == 0)
        {
            if (i != 0) // Print out Row numbers in Blue
            {
                sprintf(MLX90640_Test_Buffer, "
");
                HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
            }

            if ((i / 32) >= (*highestRowGroupStart - 1) && (i / 32) < (*highestRowGroupStart - 1 + GROUP_SIZE))
            {
                // Print in white for highest row group
                sprintf(MLX90640_Test_Buffer, "[1;37mRow %2d:[0m ", (i / 32) + 1);
            }
            else
            {
                // Print in blue for other rows
                sprintf(MLX90640_Test_Buffer, "[1;34mRow %2d:[0m ", (i / 32) + 1);
            }
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
        }

        // Check temperature ranges and add color codes accordingly
        if (mlx90640To[i] <= 26.0)
        {
            // Green color for temperature <= 26
            sprintf(MLX90640_Test_Buffer, "[32m%5.2f [0m", mlx90640To[i]);
        }
        else if (mlx90640To[i] > 26.0 && mlx90640To[i] <= 34.0)
        {
            // Yellow color for temperature between 26 and 34
            sprintf(MLX90640_Test_Buffer, "[33m%5.2f [0m", mlx90640To[i]);
        }
        else
        {
            // Red color for temperature > 34
            sprintf(MLX90640_Test_Buffer, "[31m%5.2f [0m", mlx90640To[i]);
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

        // Check for target detection
        if (mlx90640To[i] >= TARGET_MIN_TEMP && mlx90640To[i] <= TARGET_MAX_TEMP)
        {
            *targetDetected = 1;
        }
    }

    // TARGET TRIGGER
    if (*targetDetected == 1) // If-else to trigger output LED and UART print
    {
        sprintf(MLX90640_Test_Buffer, "
==========================TargetDetected==========================
");
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }

    sprintf(MLX90640_Test_Buffer, "
==========================Waveshare==========================
");
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    return *targetDetected;
}

//STM DECLARATION STUFF
void Error_Handler(void)
{
		  HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);
		  HAL_Delay (100);   /* Insert delay 100 ms */
}
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
	//GPIO SET UP

		GPIO_InitTypeDef GPIO_InitStruct = {0};
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = LD2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
		/* USER CODE BEGIN SysInit */
		MLX90640_I2CInit();
		/* USER CODE END SysInit */

		/* USER CODE BEGIN 2 */



	//CHECK FOR ANY ERROR, PRINT OUT CODE IF ANY IS FOUND

		sprintf(MLX90640_Test_Buffer, "hi2c1: %p &hi2c1: %p 
", hi2c1.Instance, &hi2c1);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		sprintf(MLX90640_Test_Buffer, "hi2c2: %p &hi2c2: %p 
", hi2c2.Instance, &hi2c2);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		sprintf(MLX90640_Test_Buffer, "hi2c3: %p &hi2c3: %p 
", hi2c3.Instance , &hi2c3);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

		// Initialize sensors
		sensors[0].hi2c = &hi2c1;
		sensors[1].hi2c = &hi2c2;
		sensors[2].hi2c = &hi2c3;

		//SET PROPER MODE FOR SENSOR 1

		MLX90640_SetRefreshRate(sensors[0].hi2c,MLX90640_ADDR, RefreshRate);
		MLX90640_SetChessMode(sensors[0].hi2c,MLX90640_ADDR);

		//SET PROPER MODE FOR SENSOR 2

		MLX90640_SetRefreshRate(sensors[1].hi2c,MLX90640_ADDR, RefreshRate);
		MLX90640_SetChessMode(sensors[1].hi2c,MLX90640_ADDR);

		//SET PROPER MODE FOR SENSOR 3

		//MLX90640_SetRefreshRate(sensors[2].hi2c,MLX90640_ADDR, RefreshRate);
		//MLX90640_SetChessMode(sensors[2].hi2c,MLX90640_ADDR);


		// Initialize other variables as needed
		for (int i = 0; i < 3; i++)
		{
		    sensors[i].targetDetected = 0;
		    sensors[i].highestRowGroupStart = 0;
		    sensors[i].highestColGroupStart = 0;
		}

	while (1)
	{
		char clear_command[] = "[2J[H"; // ANSI escape code to clear screen and move cursor to home
		HAL_UART_Transmit(&huart2, (uint8_t *)clear_command, strlen(clear_command), HAL_MAX_DELAY);

	    // Start message
	    sprintf(MLX90640_Test_Buffer, "

start
");
	    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

		for (int i = 0; i < 2; i++)//CURRENTLY SKIPING HI2C1 SO HAVE TO START AT 1, OTHERWISE START AT 0 TO INCLUDE HI2C1
		{
			sprintf(MLX90640_Test_Buffer, "
Debug info for sensor %.2d
", i);
			HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		    retriever(sensors[i].hi2c, sensors[i].eeMLX90640, &sensors[i].mlx90640, sensors[i].mlx90640To, sensors[i].frame,&sensors[i].targetDetected, &sensors[i].highestRowGroupStart, &sensors[i].highestColGroupStart);
		    sensors[i].avg = overallAvg(sensors[i].mlx90640To, &sensors[i].highestRowGroupStart, &sensors[i].highestColGroupStart);
		}

	    /* End message */
	    sprintf(MLX90640_Test_Buffer, "end
");
	    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

		// Determine the sensor with the highest average temperature
		int max_index = 0;
		float max_avg = sensors[0].avg;

		for (int i = 2; i < 2; i++)
		{
		    if (sensors[i].avg > max_avg)
		    {
		        max_avg = sensors[i].avg;
		        max_index = i;
		    }
		}
		sprintf(MLX90640_Test_Buffer, "
[32mDisplaying Sensor%2d's grid [0m
", max_index);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

		// Process the selected sensor using a switch case
		switch (max_index)
		{
/*		    case 0:
		        // Sensor 1 has the highest average temperature
		        sensors[0].highestColumnIndex = avgBySector(sensors[0].mlx90640To,&sensors[0].highestRowGroupStart,&sensors[0].highestColGroupStart);
		        printer(sensors[0].hi2c, sensors[0].eeMLX90640, &sensors[0].mlx90640,sensors[0].mlx90640To, sensors[0].frame,&sensors[0].targetDetected, &sensors[0].highestRowGroupStart, &sensors[0].highestColGroupStart);
		        setLEDState(sensors[0].targetDetected);
		        break;

		    case 1:
		        // Sensor 2 has the highest average temperature
		        sensors[1].highestColumnIndex = avgBySector(sensors[1].mlx90640To,&sensors[1].highestRowGroupStart,&sensors[1].highestColGroupStart);
		        printer(sensors[1].hi2c, sensors[1].eeMLX90640, &sensors[1].mlx90640,sensors[1].mlx90640To, sensors[1].frame,&sensors[1].targetDetected, &sensors[1].highestRowGroupStart, &sensors[1].highestColGroupStart);
		        setLEDState(sensors[1].targetDetected);
		        break;

		    case 2:
		        // Sensor 3 has the highest average temperature
		        sensors[2].highestColumnIndex = avgBySector(sensors[2].mlx90640To,&sensors[2].highestRowGroupStart,&sensors[2].highestColGroupStart);
		        printer(sensors[2].hi2c, sensors[2].eeMLX90640, &sensors[2].mlx90640,sensors[2].mlx90640To, sensors[2].frame,&sensors[2].targetDetected, &sensors[2].highestRowGroupStart, &sensors[2].highestColGroupStart);
		        setLEDState(sensors[2].targetDetected);
		        break;
*/
		    default:
		        // Handle unexpected cases if necessary
		        // Sensor 1 has the highest average temperature
		        sensors[0].highestColumnIndex = avgBySector(sensors[0].mlx90640To,&sensors[0].highestRowGroupStart,&sensors[0].highestColGroupStart);
		        printer(sensors[0].hi2c, sensors[0].eeMLX90640, &sensors[0].mlx90640,sensors[0].mlx90640To, sensors[0].frame,&sensors[0].targetDetected, &sensors[0].highestRowGroupStart, &sensors[0].highestColGroupStart);
		        setLEDState(sensors[0].targetDetected);

		        // Sensor 2 has the highest average temperature
		        sensors[1].highestColumnIndex = avgBySector(sensors[1].mlx90640To,&sensors[1].highestRowGroupStart,&sensors[1].highestColGroupStart);
		        printer(sensors[1].hi2c, sensors[1].eeMLX90640, &sensors[1].mlx90640,sensors[1].mlx90640To, sensors[1].frame,&sensors[1].targetDetected, &sensors[1].highestRowGroupStart, &sensors[1].highestColGroupStart);
		        setLEDState(sensors[1].targetDetected);
		        break;
		}
		// Optionally, control servo motors or other actuators based on Sensor 1 data
		// Servo Motor Control in a loop
		loop();
		//WheelMode(1);
		//rotateConstantSpeed(1);	    return angle;
		rotateToTargetColumn(sensors[0].highestColGroupStart, sensors[0].targetDetected);
		rotateToTargetRow(sensors[1].highestRowGroupStart, sensors[1].targetDetected);
		}
	    return 0;
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


