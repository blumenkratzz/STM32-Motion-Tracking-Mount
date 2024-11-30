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
#define  TARGET_MIN_TEMP 26
#define  TARGET_MAX_TEMP 37

// Define constants for sensor numbers
#define SENSOR_LEFT 0
#define SENSOR_MIDDLE 1
#define SENSOR_RIGHT 2

//calibration for average count thingy
#define SENSOR_ARRAY_SIZE 768
#define ROWS 24
#define COLUMNS 32


#define ROW_CHUNK 2
#define COLUMN_CHUNK 2

// Define the number of frames to combine for temporal analysis (inter-frame)
#define NUM_FRAMES_TO_COMBINE 4
#define MINIMUM_TARGET_PIXELS 2 // Adjust as needed

I2C_HandleTypeDef *hi2c;
UART_HandleTypeDef huart2;
//extern I2C_HandleTypeDef hi2c1;
//volatile I2C_HandleTypeDef hi2c1;
//volatile I2C_HandleTypeDef hi2c3;

// Servo Motor external declarations
extern void setup(void);
extern void loop(void);
extern void rotateConstantSpeed(uint8_t);
extern float rotateToTargetColumn(uint8_t, int, int);
extern float rotateToTargetRow(uint8_t, int);
extern void calibrateMotors(void);
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


int UART_ReadLine(char *buffer, int maxLength) {
    int index = 0;
    char c;

    while (index < maxLength - 1) {
        // Read one character at a time
        if (HAL_UART_Receive(&huart2, (uint8_t *)&c, 1, HAL_MAX_DELAY) == HAL_OK) {
            // Echo back the received character
            HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, HAL_MAX_DELAY);

            if (c == '' || c == '
') {
                // End of line detected
                break;
            } else if (c == '' && index > 0) {
                // Handle backspace
                index--;
                continue;
            } else {
                buffer[index++] = c;
            }
        }
    }
    buffer[index] = ' '; // Null-terminate the string
    return index;
}

// Functions to process MLX90640 data
int retriever(I2C_HandleTypeDef *hi2c, uint16_t *eeMLX90640, paramsMLX90640 *mlx90640,
              float *mlx90640To, uint16_t *frame, int *targetDetected,
              int *highestRowGroupStart, int *highestColGroupStart, int numFrames)
{
    float emissivity = 0.95; // Emissivity value
    float mlx90640ToAccum[SENSOR_ARRAY_SIZE] = {0}; // Accumulator for temperatures
    int status;
    float tr = 23.15 - TA_SHIFT; // Initialize tr with a default value

    // Read EEPROM data and extract parameters if not done already
    static int parameters_extracted = 0;
    if (!parameters_extracted)
    {
        status = MLX90640_DumpEE(hi2c, MLX90640_ADDR, eeMLX90640);
        if (status != 0)
        {
            sprintf(MLX90640_Test_Buffer, "MLX90640_DumpEE Error: %d
", status);
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
            return -1;
        }

        status = MLX90640_ExtractParameters(eeMLX90640, mlx90640);
        if (status != 0)
        {
            sprintf(MLX90640_Test_Buffer, "MLX90640_ExtractParameters Error: %d
", status);
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
            return -1;
        }
        parameters_extracted = 1;
    }

    // Accumulate temperature data over multiple frames
    for (int f = 0; f < numFrames; f++)
    {
        // Get frame data
        status = MLX90640_GetFrameData(hi2c, MLX90640_ADDR, frame);
        if (status < 0)
        {
            sprintf(MLX90640_Test_Buffer, "MLX90640_GetFrameData Error: %d
", status);
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
            return -1;
        }

        // Calculate Vdd and Ta
        float vdd = MLX90640_GetVdd(frame, mlx90640);
        float Ta = MLX90640_GetTa(frame, mlx90640);

        // Suppress unused variable warning for vdd
        (void)vdd;

        // Calculate tr (reflected temperature)
        tr = Ta - TA_SHIFT;

        // Calculate To (object temperatures)
        MLX90640_CalculateTo(frame, mlx90640, emissivity, tr, mlx90640To);

        // Check for NaN values
        int nan_found = 0;
        for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
        {
            if (isnan(mlx90640To[i]))
            {
                nan_found = 1;
                sprintf(MLX90640_Test_Buffer, "NaN found at index %d
", i);
                HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
                break;
            }
        }
        if (!nan_found)
        {
            sprintf(MLX90640_Test_Buffer, "No NaN values in mlx90640To
");
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
        }
        // Accumulate temperatures
        for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
        {
            mlx90640ToAccum[i] += mlx90640To[i];
        }
    }

    // Calculate average temperatures
    for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
    {
        mlx90640To[i] = mlx90640ToAccum[i] / numFrames;
    }

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

int avgBySector(float mlx90640To_2[SENSOR_ARRAY_SIZE], int* highestRowGroupStart, int* highestColGroupStart, int* targetDetected)
{
    // GROUP_SIZE is defined elsewhere
    float highestRowGroupAverage = -1.0;
    float highestColGroupAverage = -1.0;

    *highestRowGroupStart = -1;
    *highestColGroupStart = -1;
    *targetDetected = 0; // Initialize targetDetected

    int targetPixelCount = 0;

    // Calculate targetPixelCount
    for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
    {
        if (mlx90640To_2[i] >= TARGET_MIN_TEMP && mlx90640To_2[i] <= TARGET_MAX_TEMP)
        {
            targetPixelCount++;
        }
    }

    // Determine if target is detected based on the number of pixels meeting the threshold
    if (targetPixelCount >= MINIMUM_TARGET_PIXELS)
    {
        *targetDetected = 1;
        // TARGET TRIGGER
        sprintf(MLX90640_Test_Buffer, "
==========================TargetDetected==========================
");
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
    else
    {
        *targetDetected = 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }

    // Calculate groups of rows average
    for (int r = 0; r < ROWS; r += ROW_CHUNK)
    {
        float groupRowSum = 0.0;
        int rowsToSum = (r + ROW_CHUNK <= ROWS) ? ROW_CHUNK : (ROWS - r);
        for (int i = 0; i < rowsToSum; i++)
        {
            for (int c = 0; c < COLUMNS; c++)
            {
                groupRowSum += mlx90640To_2[(r + i) * COLUMNS + c];
            }
        }
        float groupRowAverage = groupRowSum / (rowsToSum * COLUMNS);
        if (rowsToSum * COLUMNS == 0)
        {
            groupRowAverage = 0;
        }

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

    // Calculate groups of columns average
    for (int c = 0; c < COLUMNS; c += COLUMN_CHUNK)
    {
        float groupColSum = 0.0;
        int colsToSum = (c + COLUMN_CHUNK <= COLUMNS) ? COLUMN_CHUNK : (COLUMNS - c);
        for (int i = 0; i < colsToSum; i++)
        {
            for (int r = 0; r < ROWS; r++)
            {
                groupColSum += mlx90640To_2[r * COLUMNS + (c + i)];
            }
        }
        float groupColAverage = groupColSum / (colsToSum * ROWS);

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

    // Print highest row and column group averages
    if (*highestRowGroupStart != -1)
    {
        sprintf(MLX90640_Test_Buffer, "[1;31mHighest Row Group Average: Row Group starting at Row %d with Average: %.2f[0m
",
                *highestRowGroupStart, highestRowGroupAverage);
        HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
    }
    if (*highestColGroupStart != -1)
    {
        sprintf(MLX90640_Test_Buffer, "[1;31mHighest Column Group Average: Column Group starting at Column %d with Average: %.2f[0m
",
                *highestColGroupStart, highestColGroupAverage);
        HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
    }

    // Return the starting column of the highest average column group
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
            uint16_t *frame, int *highestRowGroupStart, int *highestColGroupStart)
{
    /* Waveshare data */
    sprintf(MLX90640_Test_Buffer, "
==========================Waveshare==========================
");
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    // Print out Column numbers, starting from right (Column 1) to left (Column 32)
    sprintf(MLX90640_Test_Buffer, "     Col"); // Initial spacing for row labels
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    for (int col = 31; col >= 0; col--)
    {
        int displayCol = col + 1; // Adjust column number for display (32 down to 1)

        if (displayCol >= *highestColGroupStart && displayCol < (*highestColGroupStart + COLUMN_CHUNK))
        {
            // Print in white for highest column group
            sprintf(MLX90640_Test_Buffer, " [1;37m%2d [0m  ", displayCol);
        }
        else
        {
            // Print in blue for other columns
            sprintf(MLX90640_Test_Buffer, " [1;34m%2d [0m  ", displayCol);
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
    }
    sprintf(MLX90640_Test_Buffer, "
");
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    // Now loop over rows and columns
    for (int row = 0; row < 24; row++)
    {
        // Print row label
        if (row >= (*highestRowGroupStart - 1) && row < (*highestRowGroupStart - 1 + ROW_CHUNK))
        {
            // Print in white for highest row group
            sprintf(MLX90640_Test_Buffer, "[1;37mRow %2d:[0m ", row + 1);
        }
        else
        {
            // Print in blue for other rows
            sprintf(MLX90640_Test_Buffer, "[1;34mRow %2d:[0m ", row + 1);
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

        // Loop over columns in reverse order
        for (int col = 31; col >= 0; col--)
        {
            int index = row * 32 + col;
            int displayCol = col + 1; // Adjust column number for display (32 down to 1)

            // Check temperature ranges and add color codes accordingly
            if (mlx90640To[index] <= 26.0)
            {
                // Green color for temperature <= 26
                sprintf(MLX90640_Test_Buffer, "[32m%5.2f [0m", mlx90640To[index]);
            }
            else if (mlx90640To[index] > 26.0 && mlx90640To[index] <= 34.0)
            {
                // Yellow color for temperature between 26 and 34
                sprintf(MLX90640_Test_Buffer, "[33m%5.2f [0m", mlx90640To[index]);
            }
            else
            {
                // Red color for temperature > 34
                sprintf(MLX90640_Test_Buffer, "[31m%5.2f [0m", mlx90640To[index]);
            }
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
        }
        // End of row, print newline
        sprintf(MLX90640_Test_Buffer, "
");
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
    }

    sprintf(MLX90640_Test_Buffer, "
==========================Waveshare==========================
");
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    return 0;
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

		MLX90640_SetRefreshRate(sensors[2].hi2c,MLX90640_ADDR, RefreshRate);
		MLX90640_SetChessMode(sensors[2].hi2c,MLX90640_ADDR);




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

		for (int i = 0; i < 1; i++)//CURRENTLY SKIPING HI2C1 SO HAVE TO START AT 1, OTHERWISE START AT 0 TO INCLUDE HI2C1
		{
			sprintf(MLX90640_Test_Buffer, "
Debug info for sensor %.2d
", i);
			HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
			int status = retriever(sensors[i].hi2c, sensors[i].eeMLX90640, &sensors[i].mlx90640,
			                       sensors[i].mlx90640To, sensors[i].frame, &sensors[i].targetDetected,
			                       &sensors[i].highestRowGroupStart, &sensors[i].highestColGroupStart,
			                       NUM_FRAMES_TO_COMBINE);
			if (status != 0)
			{
			    // Handle error if needed
				sprintf(MLX90640_Test_Buffer, "Error status occured during retrieval of index %d", i);
				HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
			    continue;
			}
		}

	    /* End message */
	    sprintf(MLX90640_Test_Buffer, "end
");
	    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

		// Determine the sensor with the highest average temperature
		int max_index = 0;
		float max_avg = sensors[0].avg;

		for (int i = 0; i < 1; i++)
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
		    case 0:
		        // Sensor 1 has the highest average temperature

		        sensors[0].highestColumnIndex = avgBySector(sensors[0].mlx90640To,
		        		&sensors[0].highestRowGroupStart,&sensors[0].highestColGroupStart, &sensors[0].targetDetected);
				rotateToTargetColumn(sensors[0].highestColGroupStart, sensors[0].targetDetected, SENSOR_MIDDLE);
				//rotateToTargetRow(sensors[0].highestRowGroupStart, sensors[0].targetDetected);
		        printer(sensors[0].hi2c, sensors[0].eeMLX90640, &sensors[0].mlx90640,sensors[0].mlx90640To, sensors[0].frame, &sensors[0].highestRowGroupStart, &sensors[0].highestColGroupStart);
		        setLEDState(sensors[0].targetDetected);
		        break;

		    case 1:
		        // Sensor 2 has the highest average temperature

		        setLEDState(sensors[1].targetDetected);

		        break;

		    case 2:
		        //Sensor 3 has the highest average temperature

		        setLEDState(sensors[2].targetDetected);

		        break;

		    default:
		        // Handle unexpected cases if necessary

		        break;
		}
		// Optionally, control servo motors or other actuators based on Sensor 1 data
		// Servo Motor Control in a loop
		loop();
		//WheelMode(1);
		//rotateConstantSpeed(1);	    return angle;
		 uint8_t userInput;
		        if (HAL_UART_Receive(&huart2, &userInput, 1, 10) == HAL_OK)
		        {
		            if (userInput == 'c' || userInput == 'C')
		            {
		                calibrateMotors();
		            }
		        }
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
