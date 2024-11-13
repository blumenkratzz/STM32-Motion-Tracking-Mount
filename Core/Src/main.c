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
#define	 RefreshRate FPS16HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air
#define  TARGET_MIN_TEMP 33
#define  TARGET_MAX_TEMP 37

//calibration for average count thingy
#define NUM_SAMPLES 10
#define NUM_AVERAGES 5
#define NUM_SENSORS 1
#define SENSOR_ARRAY_SIZE 768
#define ROWS 24
#define COLUMNS 32

#define GROUP_SIZE 1//Consider Wiring this up to a potentiometer to adjust accuracy

I2C_HandleTypeDef *hi2c;
extern I2C_HandleTypeDef hi2c2;

// Servo Motor external declarations
extern void setup(void);
extern void loop(void);
extern void rotateConstantSpeed(uint8_t);
extern float rotateToTargetColumn(uint8_t);
static void MX_USART2_UART_Init(void);

//variable declaration
int targetDetected;
static uint16_t eeMLX90640[832];
float mlx90640To_1[768];
float mlx90640To_2[768];
float mlx90640To_3[768];
uint16_t frame[834];
float emissivity=0.95;//0.95 is default
int status;
char MLX90640_Test_Buffer[255];
paramsMLX90640 mlx90640; //struct found in GitHub libraries



//VERY IMPORTANT
//SENSOR_L
//PB3 --- SDA PB10 --- SCL FOR I2C2

//SENSOR_M
//PB8 --- SDA PB9 --- SCL FOR I2C1

//SENSOR_R
//PA8 --- SDA PC9 --- SCL FOR I2C3

UART_HandleTypeDef huart2;




typedef struct //struct to calibrate
{
    float samples[NUM_SAMPLES];
    float sampleAvg[NUM_AVERAGES];
    int sampleCount;
    int avgCount;
} irSensor_t;

irSensor_t irs[NUM_SENSORS];

typedef enum
{
    WAIT_FOR_INPUT,
    RUN_SENSOR_L,
	RUN_SENSOR_M,
	RUN_SENSOR_R,
    EXIT
} ProgramState;


// Function to process MLX90640 data
//CURRENTLY USING THIS FUNCTION, COMBINED ALL THE ABOVE
int averageCalcBySector(float mlx90640To_2[SENSOR_ARRAY_SIZE], int* highestRowGroupStart, int* highestColGroupStart)
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
    float sum = 0.0;
    float overallAverage;

    //TO CALCULATE OVERALL AVERAGE
    for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
    {
        sum += mlx90640To_2[i];
        overallAverage = sum / SENSOR_ARRAY_SIZE;
    }

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
	sprintf(MLX90640_Test_Buffer, "[1;31mOverall Average: %.2f[0m
", overallAverage);
	HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

	// Alex modified for working with Highest Avg Column
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

//int printer(I2C_HandleTypeDef *hi2c, int targetDetected, int highestRowGroupStart, int highestColGroupStart)
int printer(int targetDetected, int highestRowGroupStart, int highestColGroupStart)
{

	//INITIALIZATION OF THE SENSOR AND CHECK FOR ANY ERRORS BEFORE STARTING
	//ALSO PRINT OUT CURRENT SENSOR INPUT VOLTAGE AND AMBIENT TEMP FOR TROUBLE SHOOTING
				  sprintf(MLX90640_Test_Buffer, "

start
");
				  HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
						int status = MLX90640_GetFrameData(hi2c, MLX90640_ADDR, frame);
						if (status < 0)
						{
							 sprintf(MLX90640_Test_Buffer, "GetFrame Error: %d
", status);
							 HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
						}
						float vdd = MLX90640_GetVdd(frame, &mlx90640);
						float Ta = MLX90640_GetTa(frame, &mlx90640);
						float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
						//tr = Ambient Reflected Temperature
						sprintf(MLX90640_Test_Buffer, "vdd:  %f Tr: %f
",vdd,tr);
					    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
						MLX90640_CalculateTo(frame, &mlx90640, emissivity , tr, mlx90640To_2);
						 /* End message */
						sprintf(MLX90640_Test_Buffer, "end
");
					    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
					    /* Waveshare data */
						sprintf(MLX90640_Test_Buffer, "
==========================Waveshare==========================
");
						HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

	//FOR LOOP FOR PRINTING OUT VALUES STARTS HERE

						//print out Column numbers in Blue
						sprintf(MLX90640_Test_Buffer, "     Col"); // Initial spacing for row labels
						HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
						for (int col = 0; col < 32; col++)
						{
							if (col >= highestColGroupStart - 1 && col < highestColGroupStart - 1 + GROUP_SIZE)
							{
							// Print in white for highest column group
								sprintf(MLX90640_Test_Buffer, " [1;37m%2d [0m  ", col + 1);
							}
							else
							{
							// Print in blue for other columns
				            sprintf(MLX90640_Test_Buffer, " [1;34m%2d [0m  ", col + 1);
					        }
					        HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
						}
						sprintf(MLX90640_Test_Buffer, "
");
						HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

						//for(int i = 0; i < 768; i++)
						for(int i = 0; i < 768; i++)//for loop print temp variables
						{
							if (i % 32 == 0)
							{
								if (i != 0)//print out Row numbers in Blue
								{
									sprintf(MLX90640_Test_Buffer, "
");
									HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
								}
								//(i / 32) >= highestRowGroupStart - 1 && (i / 32) < highestRowGroupStart - 1 + GROUP_SIZE)
								if ((i / 32) >= highestRowGroupStart - 1 && (i / 32) < highestRowGroupStart - 1 + GROUP_SIZE)
								{
								// Print in white for highest row group
						            sprintf(MLX90640_Test_Buffer, "[1;37mRow %2d:[0m ", (i / 32) + 1);
						        }
								else
						        {
								// Print in blue for other rows
								   sprintf(MLX90640_Test_Buffer, "[1;34mRow %2d:[0m ", (i / 32) + 1);
								}
								HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
						    }

							sprintf(MLX90640_Test_Buffer, "%2.2f ", mlx90640To_2[i]);
							// Check temperature ranges and add color codes accordingly
						    if (mlx90640To_2[i] <= 26.0)
						    {
						        // Green color for temperature <= 30
						        sprintf(MLX90640_Test_Buffer, "[32m%5.2f [0m", mlx90640To_2[i]);
						    }
						    else if (mlx90640To_2[i] > 26.0 && mlx90640To_2[i] <= 34.0)
						    {
							        // Yellow color for temperature between 33 and 40
						    	sprintf(MLX90640_Test_Buffer, "[33m%5.2f [0m", mlx90640To_2[i]);
							}
							else
							{
								// Red color for temperature > 50
								sprintf(MLX90640_Test_Buffer, "[31m%5.2f [0m", mlx90640To_2[i]);
							}
						    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

	//GOES THRU EACH VALUES IN THE ARRAY TO SEE IF ANY MATCHED DESISERED TEMP

							if (mlx90640To_2[i] >= TARGET_MIN_TEMP && mlx90640To_2[i] <= TARGET_MAX_TEMP)
							{//determine whether it can detect target
								targetDetected = 1;
							}
//							else
//							{
//								targetDetected = 0;
//							}
						}
	//TARGET TRIGGER
						if (targetDetected == 1) //if else to trigger output LED and UART print
						{
							sprintf(MLX90640_Test_Buffer, "
==========================TargetDetected==========================
");
							HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
							HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
						}
						else
						{
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
						}
						sprintf(MLX90640_Test_Buffer, "
==========================Waveshare==========================
");
						HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

	//DELAY BETWEEN EACH CALL FOR THE FUNCTION

						HAL_Delay(25);
						return targetDetected;
}

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

		/* USER CODE END SysInit */
		MX_I2C1_Init();
		MX_I2C2_Init();
		MX_I2C3_Init();
		//I2C_HandleTypeDef *hi2c;
		hi2c = &hi2c2; //need at least 1 channel to pass in the address for everything to load properly

		/* USER CODE BEGIN 2 */

	//CALL IN FUNCTION FROM SENSOR LIBRARIES TO GET VALUES

	   	MLX90640_SetRefreshRate(hi2c,MLX90640_ADDR, RefreshRate);
		MLX90640_SetChessMode(hi2c,MLX90640_ADDR);

	//CHECK FOR ANY ERROR, PRINT OUT CODE IF ANY IS FOUND

		//EEPROM DATA CHECK
	    status = MLX90640_DumpEE(hi2c,MLX90640_ADDR, eeMLX90640);
		sprintf(MLX90640_Test_Buffer, "
Value of status for debug: %d
", status);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		/*if (status != 0)
		{
			sprintf(MLX90640_Test_Buffer, "
load system parameters error with code: %d
", status);
			HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		}*/

		//PARAMETER CHECK
		status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
		sprintf(MLX90640_Test_Buffer, "
New value of status for debug: %d
", status);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		/*if (status != 0)
		{
			sprintf(MLX90640_Test_Buffer, "
Parameter extraction failed with error code: %d
", status);
			HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		}*/


	//Declaration to deal with pointers
	int highestRowGroupStart;
	int highestColGroupStart;
	int highestColumnIndex=0;

	uint8_t userInput[1];  // Buffer for storing user input
	//message for user input and inform of invaild input
	char promptMessage[] = "1,2,3 for Left, Middle, Right or 'Esc' to return to this prompt:
";
	char invalidMessage[] = "Invalid input. Please enter '1' or press 'Esc' to return.
";



	//NORMAL OPERATION LOOP
		/*while(1)
		{
			//SAMPLE ORDER OF FUNCTION
			printer(targetDetected, highestRowGroupStart, highestColGroupStart);//READ DATA FROM SENSOR

			//calculateRowAndColumnAverages(mlx90640To);
			//old average row and col has been rebuilt into averageCalcBySector

			averageCalcBySector(mlx90640To, &highestRowGroupStart, &highestColGroupStart);//PROCESS DATA
			setLEDState(targetDetected);//ANALYZE DATA TRIGGER LED

		}*/
	ProgramState currentState = WAIT_FOR_INPUT;

	while (currentState != EXIT)
	{
	        switch (currentState)
	        {
	            case WAIT_FOR_INPUT:
	                // Prompt the user for input
	                HAL_UART_Transmit(&huart2, (uint8_t*)promptMessage, strlen(promptMessage), HAL_MAX_DELAY);

	                // Wait for user input over UART
	                HAL_UART_Receive(&huart2, userInput, 1, HAL_MAX_DELAY);

	                if (userInput[0] == '1') // Check if user input is '1' or 'Esc'
	                {
	                	//hi2c = &hi2c2;
	                    currentState = RUN_SENSOR_L;  // Transition to running functions
	                }
	                else if (userInput[0] == '2')
	               	{
	                    currentState = RUN_SENSOR_M;
	                }
	                else if (userInput[0] == '3')
	                {
	                	currentState = RUN_SENSOR_R;
	                }
	                else if (userInput[0] == 27)// 27 is the ASCII code for 'Esc'
	                {
	                    currentState = EXIT;  // Exit the program
	                }
	                else// Inform the user about invalid input
	                {
	                    HAL_UART_Transmit(&huart2, (uint8_t*)invalidMessage, strlen(invalidMessage), HAL_MAX_DELAY);
	                }
	                break;

	            case RUN_SENSOR_L:
	                while (1)
	                {
	                    // READ DATA FROM SENSOR
	                    printer(targetDetected, highestRowGroupStart, highestColGroupStart);

	                    // PROCESS DATA
	                    highestColumnIndex=averageCalcBySector(mlx90640To_2, &highestRowGroupStart, &highestColGroupStart);

	                    // ANALYZE DATA AND TRIGGER LED
	                    setLEDState(targetDetected);

	                    // Non-blocking check for 'Esc' to exit the function loop
	                    if (HAL_UART_Receive(&huart2, userInput, 1, 10) == HAL_OK)
	                    {
	                        if (userInput[0] == 27)
	                        {  // ASCII code for 'Esc'
	                            currentState = WAIT_FOR_INPUT;
	                            break;
	                        }
	                    }

	                    // Servo Motor Control in a loop
	            		loop();
	            		//WheelMode(1);
	            		//rotateConstantSpeed(1);	    return angle;
	            		rotateToTargetColumn(highestColumnIndex);
	                }
	                break;

	            case EXIT:
	                // Simply exit the loop and program
	                break;

	            default:
	                // Unexpected state (should not happen)
	                currentState = WAIT_FOR_INPUT;
	                break;
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
