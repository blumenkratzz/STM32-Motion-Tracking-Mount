/*
 * Authors: Aleksandr Safronov, Quoc Trung Tran
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
#include <math.h>

// MLX90640 Sensor Includes
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "i2c.h"
#include "usart.h"


// Servo Motors Includes
#include "stm32f4xx.h"
#include "SCServo.h"
#include "uart.h"

#define SENSOR_DEBUG 1

#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33 //default address
#define	 RefreshRate FPS16HZ //max speed 16
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air
#define  TARGET_MIN_TEMP 24.0f
#define  TARGET_MAX_TEMP 37.0f

// Define constants for sensor numbers
#define SENSOR_LEFT 1
#define SENSOR_MIDDLE 0
#define SENSOR_RIGHT 2
#define RETURN_HOME -1

//calibration for average count thingy
#define SENSOR_ARRAY_SIZE 768
#define ROWS 24
#define COLUMNS 32


#define ROW_CHUNK 4
#define COLUMN_CHUNK 2

// Define the number of frames to combine for temporal analysis (inter-frame)
#define NUM_FRAMES_TO_COMBINE 4
#define MINIMUM_TARGET_BLOB_SIZE 8
#define NUM_SENSORS 3

uint32_t lastDetectedTime = 0;
uint8_t stableTargetDetected = 0; // This is the final state used for actual decisions
#define LOST_TARGET_TIMEOUT_MS 2000 // 2 seconds

I2C_HandleTypeDef *hi2c;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;

// Servo Motor external declarations
extern void setupMotorControl(void);
extern void checkServoConnection(void);
extern void rotateConstantSpeed(uint8_t);
extern float rotateToTargetColumn(uint8_t, int, int);
extern float rotateToTargetRow(uint8_t, int);
extern void calibrateMotors(void);
extern uint32_t MapDelay(uint16_t);
extern uint16_t ReadPotentiometer(void);

static void MX_USART2_UART_Init(void);
float emissivity=0.95;//0.95 is default
int status;
char MLX90640_Test_Buffer[255];
float mlx90640ToAccum[SENSOR_ARRAY_SIZE] = {0}; // Accumulator for temperatures
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
    float centroidWeightedSum;
    int blobSize;
} SensorData;

SensorData sensors[3];

typedef struct {
    int zoneNumber;
    float centroidWeightedSumMin;
    float centroidWeightedSumMax;
    int headThresholdPixelCount;
} ZoneInfo;

// Define zones based on physically measured and calibrated values
ZoneInfo zones[] = {
    {1, 3500, INFINITY, 25},
    {2, 1900, 3500, 25},
    {3, 1000, 1900, 20},
    {4, 780, 1000, 18},
    {5, 600, 780, 16},
    {6, 390, 600, 14},
    {7, 290, 390, 10},
    {8, 40, 290, 4},
};

//Hardware Wiring: (FLIP SENSOR_L and SENSOR_R)

	//SENSOR_L
	//PB3 --- SDA PB10 --- SCL FOR I2C2

	//SENSOR_M
	//PB8 --- SDA PB9 --- SCL FOR I2C1

	//SENSOR_R
	//PA8 --- SDA PC9 --- SCL FOR I2C3

	// Servo Control
	//PA9 (TX) and PA10 (RX) are connected to the UART1 (ST-Link USB bridge)

	// Console SENSOR_DEBUG Output
	//PA2 (TX) and PA3 (RX) are connected/hardwired to the ST-Link/V2-1 for the Virtual COM Port (VCP)


int UART_ReadLine(char *buffer, int maxLength) {
    int index = 0;
    char c;

    while (index < maxLength - 1) {
        // Read one character at a time
        if (HAL_UART_Receive(&huart2, (uint8_t *)&c, 1, HAL_MAX_DELAY) == HAL_OK) {
            // Echo back the received character
            HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, HAL_MAX_DELAY);

            if (c == '\r' || c == '\n') {
                // End of line detected
                break;
            } else if (c == '\b' && index > 0) {
                // Handle backspace
                index--;
                continue;
            } else {
                buffer[index++] = c;
            }
        }
    }
    buffer[index] = '\0'; // Null-terminate the string
    return index;
}

// Functions to process MLX90640 data
int retriever(I2C_HandleTypeDef *hi2c, uint16_t *eeMLX90640, paramsMLX90640 *mlx90640,
              float *mlx90640To, uint16_t *frame, int *targetDetected,
              int *highestRowGroupStart, int *highestColGroupStart, int numFrames)
{
    float emissivity = 0.95; // Emissivity value
    int status;
    // Frame Accumulator Array for temperatures
    // Reset global accumulation array to 0
    memset(mlx90640ToAccum, 0, sizeof(mlx90640ToAccum));
    float tr = 23.15 - TA_SHIFT; // Initialize tr with a default value

    // Read EEPROM data and extract parameters if not done already
    static int parameters_extracted[3] = {0}; // Separate flags for each sensor
       int sensor_index = (hi2c == &hi2c1) ? SENSOR_LEFT : (hi2c == &hi2c2) ? SENSOR_MIDDLE : SENSOR_RIGHT;
    if (!parameters_extracted[sensor_index])
    {
        status = MLX90640_DumpEE(hi2c, MLX90640_ADDR, eeMLX90640);
        if (status != 0)
        {
            sprintf(MLX90640_Test_Buffer, "MLX90640_DumpEE Error: %d\r\n", status);
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
            return -1;
        }

        status = MLX90640_ExtractParameters(eeMLX90640, mlx90640);
        if (status != 0)
        {
            sprintf(MLX90640_Test_Buffer, "MLX90640_ExtractParameters Error: %d\r\n", status);
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
            return -1;
        }
        parameters_extracted[sensor_index] = 1;
    }

    // Accumulate temperature data over multiple frames
    for (int f = 0; f < numFrames; f++)
    {
        // Get frame data
        status = MLX90640_GetFrameData(hi2c, MLX90640_ADDR, frame);
        if (status < 0)
        {
            sprintf(MLX90640_Test_Buffer, "MLX90640_GetFrameData Error: %d\r\n", status);
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
            return -1;
        }

        // Calculate Vdd and Ta
        float vdd = MLX90640_GetVdd(frame, mlx90640);
        float Ta = MLX90640_GetTa(frame, mlx90640);


        // Calculate tr (reflected temperature)
        tr = Ta - TA_SHIFT;

        sprintf(MLX90640_Test_Buffer, "Vdd: %.3f, Ta: %.3f, Tr: %.3f\r\n", vdd, Ta, tr);
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

        // Calculate To (object temperatures)
        MLX90640_CalculateTo(frame, mlx90640, emissivity, tr, mlx90640To);

        // Check for NaN values
        int nan_found = 0;
        for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
        {
            if (isnan(mlx90640To[i]))
            {
                nan_found = 1;
                sprintf(MLX90640_Test_Buffer, "NaN found at index %d\r\n", i);
                HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
                break;
            }
        }
        if (!nan_found)
        {
            sprintf(MLX90640_Test_Buffer, "No NaN values in mlx90640To\r\n");
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


float getPixelWeight(float temperature)
{
    float centerTemp = 26.0f; // Center of the Gaussian
    float sigma = 2.0f;       // Standard deviation

    // Calculate Gaussian weight
    float weight = exp(-0.5 * pow((temperature - centerTemp) / sigma, 2));

    return weight;
}

float overallAvg(float mlx90640To_2[SENSOR_ARRAY_SIZE])
{
    float sum = 0.0;

    // Calculate overall sum
    for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
    {
        sum += mlx90640To_2[i];
    }
    // Calculate overall average
    float overallAverage = sum / SENSOR_ARRAY_SIZE;

    // Optionally, print the overall average
#ifdef SENSOR_DEBUG
    sprintf(MLX90640_Test_Buffer, "Overall Average: %.2f\n\r", overallAverage);
    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
#endif

    return overallAverage;
}
int avgBySector(float mlx90640To_2[SENSOR_ARRAY_SIZE], int* highestRowGroupStart, int* highestColGroupStart, int* targetDetected, int headThresholdPixelCount, int zoneInfo)
{
    *highestRowGroupStart = -1;
    *highestColGroupStart = -1;
    *targetDetected = 0; // Initialize targetDetected

    // Sliding window for rows
    int maxRowStart = ROWS - ROW_CHUNK; // Max row index to start the chunk

    for (int r = 0; r <= maxRowStart; r++)
    {
        int pixelCountInGroup = 0;

        // For each row in the group
        for (int i = 0; i < ROW_CHUNK; i++)
        {
            int rowIndex = r + i;
            // For each column
            for (int c = 0; c < COLUMNS; c++)
            {
                int index = rowIndex * COLUMNS + c;
                float temp = mlx90640To_2[index];
                if (temp >= TARGET_MIN_TEMP && temp <= TARGET_MAX_TEMP)
                {
                    pixelCountInGroup++;
                }
            }
        }

        // Check if this group meets the headThresholdPixelCount
        if ((pixelCountInGroup >= headThresholdPixelCount) && (zoneInfo != -1))
        {
            *targetDetected = 1;
            *highestRowGroupStart = r + 1; // Adjusting for 1-based indexing
            // You can break out of the loop since the target is found
            break;
        }
        else
        {
            *targetDetected = 0;
        }
    }

    // Column sliding window logic
    int maxColStart = COLUMNS - COLUMN_CHUNK; // Max column index to start the chunk
    float highestDetectionMetric = -1.0;

    for (int c = 0; c <= maxColStart; c++)
    {
        float weightedSum = 0.0;
        int blobSize = 0;

        // Process each column in the group
        for (int i = 0; i < COLUMN_CHUNK; i++)
        {
            int colIndex = c + i;
            for (int r = 0; r < ROWS; r++)
            {
                int index = r * COLUMNS + colIndex;
                float temp = mlx90640To_2[index];

                if (temp >= TARGET_MIN_TEMP)
                {
                    // Add to weighted sum using the pixel's weight
                    float weight = getPixelWeight(temp);
                    weightedSum += weight * temp;
                    blobSize++;
                }
            }
        }

        // Calculate detection metric (combined weighted sum and blob size)
        float detectionMetric = weightedSum * blobSize;

        // Check if this column group has the highest metric
        if (detectionMetric > highestDetectionMetric)
        {
            highestDetectionMetric = detectionMetric;
            *highestColGroupStart = c + 1; // Adjusting for 1-based indexing
        }
    }

    // Return the starting column of the highest detection metric column group
    return *highestColGroupStart;
}




void connectedComponents(int binaryMask[ROWS][COLUMNS], int labels[ROWS][COLUMNS], int *numComponents)
{
    int label = 0;
    int dx[4] = {-1, 1, 0, 0}; // Left, Right, Up, Down
    int dy[4] = {0, 0, -1, 1};

    // Initialize labels array
    memset(labels, 0, sizeof(int) * ROWS * COLUMNS);

    for (int r = 0; r < ROWS; r++)
    {
        for (int c = 0; c < COLUMNS; c++)
        {
            if (binaryMask[r][c] == 1 && labels[r][c] == 0)
            {
                label++;
                // BFS queue
                int queue_r[ROWS * COLUMNS];
                int queue_c[ROWS * COLUMNS];
                int front = 0;
                int back = 0;

                // Enqueue current pixel
                queue_r[back] = r;
                queue_c[back] = c;
                back++;
                labels[r][c] = label;

                while (front < back)
                {
                    int cr = queue_r[front];
                    int cc = queue_c[front];
                    front++;

                    // Check neighbors
                    for (int i = 0; i < 4; i++)
                    {
                        int nr = cr + dy[i];
                        int nc = cc + dx[i];

                        if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLUMNS)
                        {
                            if (binaryMask[nr][nc] == 1 && labels[nr][nc] == 0)
                            {
                                // Enqueue neighbor
                                queue_r[back] = nr;
                                queue_c[back] = nc;
                                back++;
                                labels[nr][nc] = label;
                            }
                        }
                    }
                }
            }
        }
    }

    *numComponents = label;
}


float calculateCentroidWeightedSum(float mlx90640To_2[SENSOR_ARRAY_SIZE], int *blobSize)
{
    // Step 1: Create binary mask
    int binaryMask[ROWS][COLUMNS] = {0};
    for (int r = 0; r < ROWS; r++)
    {
        for (int c = 0; c < COLUMNS; c++)
        {
            int index = r * COLUMNS + c;
            if (mlx90640To_2[index] >= TARGET_MIN_TEMP)
            {
                binaryMask[r][c] = 1;
            }
        }
    }

    // Step 2: Perform connected component labeling
    int labels[ROWS][COLUMNS];
    int numComponents = 0;
    connectedComponents(binaryMask, labels, &numComponents);

    if (numComponents == 0)
    {
        // No blobs found
        return 0.0f;
    }

    // Step 3: Identify the largest blob
    int componentSizes[numComponents + 1]; // Index 0 unused
    memset(componentSizes, 0, sizeof(int) * (numComponents + 1));

    for (int r = 0; r < ROWS; r++)
    {
        for (int c = 0; c < COLUMNS; c++)
        {
            int labelValue = labels[r][c];
            if (labelValue > 0)
            {
                componentSizes[labelValue]++;
            }
        }
    }

    int largestComponentLabel = 1;
    int largestComponentSize = componentSizes[1];

    for (int i = 2; i <= numComponents; i++)
    {
        if (componentSizes[i] > largestComponentSize)
        {
            largestComponentSize = componentSizes[i];
            largestComponentLabel = i;
        }
    }

    // Step 4: Calculate weighted sum
    float weightedSum = 0.0f;
    int countTemperatures = 0;

    for (int r = 0; r < ROWS; r++)
    {
        for (int c = 0; c < COLUMNS; c++)
        {
            if (labels[r][c] == largestComponentLabel)
            {
                int index = r * COLUMNS + c;
                float temperature = mlx90640To_2[index];
                float weight = getPixelWeight(temperature);

                weightedSum += weight * temperature;
                countTemperatures++;
            }
        }
    }
    // Return the size of the largest blob
       *blobSize = largestComponentSize;

    return weightedSum;
}

int getZoneInfo(float centroidWeightedSum, int *headThresholdPixelCount, int *targetDetected, int *blobSize)
{
    int numZones = sizeof(zones) / sizeof(zones[0]);

    for (int i = 0; i < numZones; i++)
    {
        if (centroidWeightedSum >= zones[i].centroidWeightedSumMin && centroidWeightedSum < zones[i].centroidWeightedSumMax)
        {
            *headThresholdPixelCount = zones[i].headThresholdPixelCount;

            // Check blob size requirement for the zone
            if (*blobSize >= MINIMUM_TARGET_BLOB_SIZE)
            {
                *targetDetected = 1; // Target is valid
                return zones[i].zoneNumber; // Return zone number
            }
            else
            {
                *targetDetected = 0; // Target invalid due to insufficient blob size
                return -1;
            }


        }
    }

    // If centroidWeightedSum is out of range or blob size is too small
    *headThresholdPixelCount = 0;
    *targetDetected = 0;
    return -1; // Indicates outside detection range
}



void setLEDState(int zoneInfo) // Example: PA5 used for LED
{

	if (zoneInfo != -1)
		{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
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
    sprintf(MLX90640_Test_Buffer, "\r\n==========================Display Grid==========================\r\n");
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
            sprintf(MLX90640_Test_Buffer, " \033[1;37m%2d \033[0m  ", displayCol);
        }
        else
        {
            // Print in blue for other columns
            sprintf(MLX90640_Test_Buffer, " \033[1;34m%2d \033[0m  ", displayCol);
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
    }
    sprintf(MLX90640_Test_Buffer, "\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    // Now loop over rows and columns
    for (int row = 0; row < ROWS; row++)
    {
        // Check if the current row is within the target row group
        if (*highestRowGroupStart != -1 && row >= (*highestRowGroupStart - 1) && row < (*highestRowGroupStart - 1 + ROW_CHUNK))
        {
            // Print in white for target row group
            sprintf(MLX90640_Test_Buffer, "\033[1;37mRow %2d:\033[0m ", row + 1);
        }
        else
        {
            // Print in blue for other rows
            sprintf(MLX90640_Test_Buffer, "\033[1;34mRow %2d:\033[0m ", row + 1);
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

        // Loop over columns in reverse order
        for (int col = 31; col >= 0; col--)
        {
            int index = row * COLUMNS + col;
            float temp = mlx90640To[index];

            // Check temperature ranges and add color codes accordingly
            if (temp <= TARGET_MIN_TEMP)
            {
                // Green color for temperature <= 24
                sprintf(MLX90640_Test_Buffer, "\033[32m%5.2f \033[0m", temp);
            }
            else if (temp > TARGET_MIN_TEMP && temp <= 34.0)
            {
                // Yellow color for temperature between 24 and 34
                sprintf(MLX90640_Test_Buffer, "\033[33m%5.2f \033[0m", temp);
            }
            else
            {
                // Red color for temperature > 34
                sprintf(MLX90640_Test_Buffer, "\033[31m%5.2f \033[0m", temp);
            }
            HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
        }
        // End of row, print newline
        sprintf(MLX90640_Test_Buffer, "\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
    }

    sprintf(MLX90640_Test_Buffer, "\r\n==========================Display Grid==========================\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

    return 0;
}




//STM DECLARATION STUFF
void Error_Handler(void)
{
		  //HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);
		  //HAL_Delay (100);   /* Insert delay 100 ms */
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
void initSystem(void)
{
	HAL_Init();
	SystemClock_Config();
	 // Enable clocks before initializing peripherals
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE(); // For servo communication
	__HAL_RCC_USART2_CLK_ENABLE(); // For console output

	MX_USART2_UART_Init(); // Initialize UART2 for console output
	MLX90640_I2CInit();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	setupMotorControl();

}

int main()
{
	//System Initialization, GPIO Configuration, Peripheral Setup
		initSystem();

	//CHECK FOR ANY HI2C Pointers mismatch,

		sprintf(MLX90640_Test_Buffer, "\rhi2c1: %p &hi2c1: %p \r\n", hi2c1.Instance, &hi2c1);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		sprintf(MLX90640_Test_Buffer, "\rhi2c2: %p &hi2c2: %p \r\n", hi2c2.Instance, &hi2c2);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		sprintf(MLX90640_Test_Buffer, "\rhi2c3: %p &hi2c3: %p \r\n", hi2c3.Instance , &hi2c3);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

		// Initialize sensors
		sensors[SENSOR_LEFT].hi2c = &hi2c1;
		sensors[SENSOR_MIDDLE].hi2c = &hi2c2;
		sensors[SENSOR_RIGHT].hi2c = &hi2c3;

		//SET PROPER MODE FOR SENSOR 1

		MLX90640_SetRefreshRate(sensors[SENSOR_LEFT].hi2c,MLX90640_ADDR, RefreshRate);
		MLX90640_SetChessMode(sensors[SENSOR_LEFT].hi2c,MLX90640_ADDR);

		//SET PROPER MODE FOR SENSOR 2

		MLX90640_SetRefreshRate(sensors[SENSOR_MIDDLE].hi2c,MLX90640_ADDR, RefreshRate);
		MLX90640_SetChessMode(sensors[SENSOR_MIDDLE].hi2c,MLX90640_ADDR);

		//SET PROPER MODE FOR SENSOR 3

		MLX90640_SetRefreshRate(sensors[SENSOR_RIGHT].hi2c,MLX90640_ADDR, RefreshRate);
		MLX90640_SetChessMode(sensors[SENSOR_RIGHT].hi2c,MLX90640_ADDR);

	    int headThresholdPixelCount = 0;
	    float centroidWeightedSum = 0;
	    int targetZone = -1;
	    int blobSize = 0;
		uint16_t adc_value = 0;
		uint32_t masterTrackingDelay = 0;

		// Initialize other variables as needed
		for (int i = 1; i < NUM_SENSORS; i++)
		{
		    sensors[i].targetDetected = 0;
		    sensors[i].highestRowGroupStart = 0;
		    sensors[i].highestColGroupStart = 0;
		}

	while (1)
	{
		char clear_command[] = "\033[2J\033[H"; // ANSI escape code to clear screen and move cursor to home
		HAL_UART_Transmit(&huart2, (uint8_t *)clear_command, strlen(clear_command), HAL_MAX_DELAY);

	    // Start message
	    sprintf(MLX90640_Test_Buffer, "\n\nstart\r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

		for (int i = 1; i < NUM_SENSORS; i++)//CURRENTLY SKIPING HI2C1 SO HAVE TO START AT 1, OTHERWISE START AT 0 TO INCLUDE HI2C1
		{
#ifdef SENSOR_DEBUG
			sprintf(MLX90640_Test_Buffer, "\r\nSENSOR_DEBUG info for sensor %.2d\r\n", i);
			HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
#endif
			//HAL_I2C_MspDeInit(&hi2c1);
			//HAL_Delay(20);
			//HAL_I2C_MspInit(&hi2c1);
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
			 // Compute overall average and assign to sensors[i].avg
			    //sensors[i].avg = overallAvg(sensors[i].mlx90640To);
#ifdef SENSOR_DEBUG
			    // Optionally print the average for this sensor
			    sprintf(MLX90640_Test_Buffer, "Sensor %d Average Temperature: %.2f\n\r", i, sensors[i].avg);
			    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
#endif
			    // Calculate weighted sum of the centroid
			    centroidWeightedSum = calculateCentroidWeightedSum(sensors[i].mlx90640To, &sensors[i].blobSize);
			    sensors[i].centroidWeightedSum = centroidWeightedSum;
			    blobSize = sensors[i].blobSize;

#ifdef SENSOR_DEBUG
			       sprintf(MLX90640_Test_Buffer, "Sensor %d Centroid Weighted Sum: %.2f, Blob Size: %d\n\r", i, centroidWeightedSum, sensors[i].blobSize);
			       HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
#endif

		}

		// Determine the sensor with the highest centroid weighted sum
		// Currently, blobSize (clustering) is part of detection scheme, Might be improved to be a combined factor using coef-s.
		// detectionFactor = centroidWeightedSum + (blobSize*centroidWeightedSum*0.1)
		// 12 pixels, with 3 pixel blob size each (e.g. 4 people), VS. 10 pixel of single person (blob)
		// detectionFactor= a + 0.1*a*3 = 1.3, VS. b + 0.1*b*10 = 2, Assuming a=b where a, b, are respective centroids of each blob
		// Or you can see with increasing a, that b still takes over due to closer clustering (but up to a point)
		int max_index = 0;
		float max_centroid_weighted_sum = 0;
		int localTargetDetected = 0;
		float detectionMetric = 0;
		for (int i = 1; i < NUM_SENSORS; i++)
		{
		    /*if (sensors[i].centroidWeightedSum > max_centroid_weighted_sum)
		    {
		    	max_centroid_weighted_sum = sensors[i].centroidWeightedSum;
		        max_index = i;
#ifdef SENSOR_DEBUG
				sprintf(MLX90640_Test_Buffer, "\r\nSensor %d Found Higher Centroid Weighted Sum: %.2f\r\n", max_index, sensors[max_index].centroidWeightedSum);
				HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
#endif
		    }*/

		    if ((sensors[i].centroidWeightedSum * sensors[i].blobSize) > detectionMetric)
		    {
				detectionMetric=sensors[i].centroidWeightedSum * sensors[i].blobSize;
		        max_index = i;
#ifdef SENSOR_DEBUG
				sprintf(MLX90640_Test_Buffer, "\r\nSensor %d Found Higher detectionMetric (Centroid Weighted Sum * Blob Size): %.2f\r\n", max_index, sensors[max_index].centroidWeightedSum);
				HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
#endif
		    }
		}

#ifdef SENSOR_DEBUG
		// Define a variable to store the sensor direction as a string
		char sensor_direction[20];

		// Assign the direction based on max_index
		if (max_index == SENSOR_LEFT) {
		    strcpy(sensor_direction, "SENSOR_LEFT");
		} else if (max_index == SENSOR_RIGHT) {
		    strcpy(sensor_direction, "SENSOR_RIGHT");
		} else if (max_index == SENSOR_MIDDLE) {
		    strcpy(sensor_direction, "SENSOR_MIDDLE");
		} else {
		    strcpy(sensor_direction, "UNKNOWN");
		}
		sprintf(MLX90640_Test_Buffer, "\r\n\033[32mDisplaying Sensor%2d's grid (%s) \033[0m\r\n", max_index, sensor_direction);
		HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
#endif


		// Read potentiometer value
		adc_value = ReadPotentiometer();

#ifdef SENSOR_DEBUG
		sprintf(MLX90640_Test_Buffer, "Raw ADC Value: %u\r\n", adc_value);
		HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
#endif
		// Map ADC value to delay
		masterTrackingDelay = MapDelay(adc_value);

#ifdef SENSOR_DEBUG
		// Debug: Print ADC value and calculated delay
		sprintf(MLX90640_Test_Buffer, "ADC: %u, Delay: %lu ms\n\r", adc_value, masterTrackingDelay);
		HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
#endif

		// Process the selected sensor using a switch case
		switch (max_index)
		{
		    case SENSOR_LEFT:
		        // Sensor 1 has the highest average temperature

			    targetZone = getZoneInfo(sensors[SENSOR_LEFT].centroidWeightedSum, &headThresholdPixelCount, &sensors[SENSOR_LEFT].targetDetected, &sensors[SENSOR_LEFT].blobSize);
			    sprintf(MLX90640_Test_Buffer, "Sensor %d Zone: %d\n\r", SENSOR_LEFT, targetZone);
			    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

			    sensors[SENSOR_LEFT].highestColumnIndex = avgBySector(sensors[SENSOR_LEFT].mlx90640To,
		    	        &sensors[SENSOR_LEFT].highestRowGroupStart, &sensors[SENSOR_LEFT].highestColGroupStart, &sensors[SENSOR_LEFT].targetDetected, headThresholdPixelCount, targetZone);

			    // After calling getZoneInfo(...), avgBySector(...)
			    // Suppose localTargetDetected is what avgBySector or getZoneInfo gave you:
			    localTargetDetected = sensors[max_index].targetDetected;

			    if (localTargetDetected == 1)
			    {
			        // Target found now, reset timer and ensure stableTargetDetected = 1
			        lastDetectedTime = HAL_GetTick();
			        stableTargetDetected = 1;
			    }
			    else
			    {
			        // Target not found this frame
			        uint32_t currentTime = HAL_GetTick();
			        if ((currentTime - lastDetectedTime) >= LOST_TARGET_TIMEOUT_MS)
			        {
			            // More than 3 seconds have passed without detecting target
			            stableTargetDetected = 0;
			        }
			            // Less than 3 seconds, keep stableTargetDetected = 1
			    }

			    // Apply Master Tracking Delay
				HAL_Delay(masterTrackingDelay);

				if (stableTargetDetected == 1)
				{
				    // Proceed with rotateToTargetRow/Column
			    rotateToTargetColumn(sensors[SENSOR_LEFT].highestColGroupStart, targetZone, SENSOR_LEFT);
			    rotateToTargetRow(sensors[SENSOR_LEFT].highestRowGroupStart, targetZone);
				}
				else{
			    	rotateToTargetColumn(sensors[SENSOR_LEFT].highestColGroupStart, targetZone, RETURN_HOME);
				    rotateToTargetRow(sensors[SENSOR_LEFT].highestRowGroupStart, RETURN_HOME);
				}
#ifdef SENSOR_DEBUG
			    printer(sensors[SENSOR_LEFT].hi2c, sensors[SENSOR_LEFT].eeMLX90640, &sensors[SENSOR_LEFT].mlx90640,sensors[SENSOR_LEFT].mlx90640To, sensors[SENSOR_LEFT].frame, &sensors[SENSOR_LEFT].highestRowGroupStart, &sensors[SENSOR_LEFT].highestColGroupStart);
#endif
		        setLEDState(targetZone);
		        break;

		    case SENSOR_MIDDLE:
		        // Sensor 2 has the highest average temperature
			    targetZone= getZoneInfo(sensors[SENSOR_MIDDLE].centroidWeightedSum, &headThresholdPixelCount, &sensors[SENSOR_MIDDLE].targetDetected, &sensors[SENSOR_MIDDLE].blobSize);
			    sprintf(MLX90640_Test_Buffer, "Sensor %d Zone: %d\n\r", SENSOR_MIDDLE, targetZone);
			    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);
		    	sensors[SENSOR_MIDDLE].highestColumnIndex = avgBySector(sensors[SENSOR_MIDDLE].mlx90640To,
		    	        &sensors[SENSOR_MIDDLE].highestRowGroupStart, &sensors[SENSOR_MIDDLE].highestColGroupStart, &sensors[SENSOR_MIDDLE].targetDetected, headThresholdPixelCount, targetZone);

			    // After calling getZoneInfo(...), avgBySector(...)
			    // Suppose localTargetDetected is what avgBySector or getZoneInfo gave you:
			    localTargetDetected = sensors[max_index].targetDetected;

			    if (localTargetDetected == 1)
			    {
			        // Target found now, reset timer and ensure stableTargetDetected = 1
			        lastDetectedTime = HAL_GetTick();
			        stableTargetDetected = 1;
			    }
			    else
			    {
			        // Target not found this frame
			        uint32_t currentTime = HAL_GetTick();
			        if ((currentTime - lastDetectedTime) >= LOST_TARGET_TIMEOUT_MS)
			        {
			            // More than 3 seconds have passed without detecting target
			            stableTargetDetected = 0;
			        }
			            // Less than 3 seconds, keep stableTargetDetected = 1
			    }
		    	// Apply Master Tracking Delay
				HAL_Delay(masterTrackingDelay);

				if (stableTargetDetected == 1)
				{
				    // Proceed with rotateToTargetRow/Column
		    	rotateToTargetColumn(sensors[SENSOR_MIDDLE].highestColGroupStart, targetZone, SENSOR_MIDDLE);
			    rotateToTargetRow(sensors[SENSOR_MIDDLE].highestRowGroupStart, targetZone);
				}
				else{
			    	rotateToTargetColumn(sensors[SENSOR_MIDDLE].highestColGroupStart, targetZone, RETURN_HOME);
				    rotateToTargetRow(sensors[SENSOR_MIDDLE].highestRowGroupStart, RETURN_HOME);
				}
#ifdef SENSOR_DEBUG
		        printer(sensors[SENSOR_MIDDLE].hi2c, sensors[SENSOR_MIDDLE].eeMLX90640, &sensors[SENSOR_MIDDLE].mlx90640,sensors[SENSOR_MIDDLE].mlx90640To, sensors[SENSOR_MIDDLE].frame, &sensors[SENSOR_MIDDLE].highestRowGroupStart, &sensors[SENSOR_MIDDLE].highestColGroupStart);
#endif
		        setLEDState(targetZone);
		        break;

		    case SENSOR_RIGHT:
		        //Sensor 3 has the highest average temperature
			    targetZone= getZoneInfo(sensors[SENSOR_RIGHT].centroidWeightedSum, &headThresholdPixelCount, &sensors[SENSOR_RIGHT].targetDetected, &sensors[SENSOR_RIGHT].blobSize);
			    sprintf(MLX90640_Test_Buffer, "Sensor %d Zone: %d\n\r", SENSOR_RIGHT, targetZone);
			    HAL_UART_Transmit(&huart2, (uint8_t*)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

		    	sensors[SENSOR_RIGHT].highestColumnIndex = avgBySector(sensors[SENSOR_RIGHT].mlx90640To,
		    	        &sensors[SENSOR_RIGHT].highestRowGroupStart, &sensors[SENSOR_RIGHT].highestColGroupStart, &sensors[SENSOR_RIGHT].targetDetected, headThresholdPixelCount, targetZone);

			    // After calling getZoneInfo(...), avgBySector(...)
			    // Suppose localTargetDetected is what avgBySector or getZoneInfo gave you:
			    localTargetDetected = sensors[max_index].targetDetected;

			    if (localTargetDetected == 1)
			    {
			        // Target found now, reset timer and ensure stableTargetDetected = 1
			        lastDetectedTime = HAL_GetTick();
			        stableTargetDetected = 1;
			    }
			    else
			    {
			        // Target not found this frame
			        uint32_t currentTime = HAL_GetTick();
			        if ((currentTime - lastDetectedTime) >= LOST_TARGET_TIMEOUT_MS)
			        {
			            // More than 3 seconds have passed without detecting target
			            stableTargetDetected = 0;
			        }
			            // Less than 3 seconds, keep stableTargetDetected = 1
			    }
		    	// Apply Master Tracking Delay
				HAL_Delay(masterTrackingDelay);

				if (stableTargetDetected == 1)
				{
				    // Proceed with rotateToTargetRow/Column
				rotateToTargetColumn(sensors[SENSOR_RIGHT].highestColGroupStart, targetZone, SENSOR_RIGHT);
			    rotateToTargetRow(sensors[SENSOR_RIGHT].highestRowGroupStart, targetZone);
				}
				else{
			    	rotateToTargetColumn(sensors[SENSOR_RIGHT].highestColGroupStart, targetZone, RETURN_HOME);
				    rotateToTargetRow(sensors[SENSOR_RIGHT].highestRowGroupStart, RETURN_HOME);
				}
#ifdef SENSOR_DEBUG
			    printer(sensors[SENSOR_RIGHT].hi2c, sensors[SENSOR_RIGHT].eeMLX90640, &sensors[SENSOR_RIGHT].mlx90640,sensors[SENSOR_RIGHT].mlx90640To, sensors[SENSOR_RIGHT].frame, &sensors[SENSOR_RIGHT].highestRowGroupStart, &sensors[SENSOR_RIGHT].highestColGroupStart);
#endif
		        setLEDState(targetZone);

		        break;

		    default:
		        // Handle unexpected cases if necessary

		        break;
		}

		// Ping each servo motor to verify communication
		checkServoConnection();
		//WheelMode(1);
		//rotateConstantSpeed(1);

	    /* End message */
	    sprintf(MLX90640_Test_Buffer, "end\r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t *)MLX90640_Test_Buffer, strlen(MLX90640_Test_Buffer), HAL_MAX_DELAY);

#ifdef SENSOR_DEBUG
		uint8_t userInput;
		        if (HAL_UART_Receive(&huart2, &userInput, 1, 10) == HAL_OK)
		        {
		            if (userInput == 'c' || userInput == 'C')
		            {
		                calibrateMotors();
		            }
		        }
#endif

		}



	    return 0;
}

static void MX_USART2_UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};


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
