/*
 * Authors: Aleksandr Safronov, Quoc Trung Trun
 * File: RS485-TTL_MotorControl.c
 * Description:
 *   This file implements motor control routines for the RS485-TTL interface,
 *   including UART communication, positional logic, and ADC-based speed control.  
 *
 *   Software Dependencies:
 *     - SCSLib: A library for controlling servo motors via UART.
 *
 * License:
 *   Licensed under CC BY-NC 4.0. See LICENSE file for details.
 *   Copyright © 2024 Aleksandr Safronov, Quoc Trung Trun
 *
 * Date:
 *   Last Modified: 2024-12-05
 *
 */
/*
Ping指令测试,测试总线上相应ID舵机是否就绪,广播指令只适用于总线只有一个舵机情况
*/

#include <stdio.h>
#include "stm32f4xx.h"
#include "SCServo.h"
#include "uart.h"
#include <string.h>
#include <math.h>

// Define constants for sensor numbers
#define SENSOR_LEFT 0
#define SENSOR_MIDDLE 1
#define SENSOR_RIGHT 2

// Define constants for columns, rows and field of view
#define COLUMNS 32
#define ROWS 24
// Define motor position ranges and angles for each sensor
// Middle Sensor Calibration Data
#define MOTOR_POSITION_MIDDLE_MIN 1050   // Corresponds to column 31-32 (leftmost)
#define MOTOR_POSITION_MIDDLE_CENTER 1400 // Corresponds to column 15-16 (center)
#define MOTOR_POSITION_MIDDLE_MAX 1820   // Corresponds to column 1-2 (rightmost)
#define MOTOR_ANGLE_MIDDLE_MIN -27.5f
#define MOTOR_ANGLE_MIDDLE_MAX 27.5f

// Left Sensor Calibration Data
#define MOTOR_POSITION_LEFT_MIN 280      // Leftmost position (1050 - 770)
#define MOTOR_POSITION_LEFT_CENTER 665   // Center position
#define MOTOR_POSITION_LEFT_MAX 1050     // Edge of middle sensor's leftmost position
#define MOTOR_ANGLE_LEFT_MIN -82.5f      // -27.5 - 55
#define MOTOR_ANGLE_LEFT_MAX -27.5f

// Right Sensor Calibration Data
#define MOTOR_POSITION_RIGHT_MIN 1820    // Edge of middle sensor's rightmost position
#define MOTOR_POSITION_RIGHT_CENTER 2205 // Center position
#define MOTOR_POSITION_RIGHT_MAX 2590    // Rightmost position (1820 + 770)
#define MOTOR_ANGLE_RIGHT_MIN 27.5f
#define MOTOR_ANGLE_RIGHT_MAX 82.5f      // 27.5 + 55

extern UART_HandleTypeDef huart2; // For console output
extern UART_HandleTypeDef huart1; // For servo communication
int Motor_ID_Horizontal = -1;
int Motor_ID_Vertical = -1;

char buffer[255]; // Buffer to hold the string to transmit

void setup()
{
  Uart_Init(1000000);

  //Uart_Init(115200);
  HAL_Delay(1000);
  //unLockEprom(1);//打开EPROM保存功能
  //writeByte(1, SMS_STS_ID, 2);//ID
  //LockEprom(2);//关闭EPROM保存功能

}
// Function to read a line from UART

void calibrateMotors(void) {
    char inputBuffer[32];
    int position;
    uint8_t motorID;
    uint16_t speed = 2250; // Keep speed and acceleration the same
    uint8_t acceleration = 40;

    // Display instructions
    sprintf(buffer, "
Entering Calibration Mode.
");
    sprintf(buffer + strlen(buffer), "Enter 'h' to calibrate horizontal motor, 'v' for vertical motor, or 'q' to quit.
");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    while (1) {
        // Prompt for motor selection
        sprintf(buffer, "Select motor ('h' for horizontal, 'v' for vertical, 'q' to quit): ");
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

        UART_ReadLine(inputBuffer, sizeof(inputBuffer));

        if (inputBuffer[0] == 'h') {
            motorID = Motor_ID_Horizontal;
        } else if (inputBuffer[0] == 'v') {
            motorID = Motor_ID_Vertical;
        } else if (inputBuffer[0] == 'q') {
            // Exit calibration mode
            sprintf(buffer, "
Exiting Calibration Mode.
");
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            break;
        } else {
            sprintf(buffer, "
Invalid selection. Please try again.
");
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            continue;
        }

        // Prompt for desired position
        sprintf(buffer, "
Enter desired position (0 to 4095): ");
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

        UART_ReadLine(inputBuffer, sizeof(inputBuffer));

        // Convert input to integer
        position = atoi(inputBuffer);

        // Validate position
        if (position < 0 || position > 4095) {
            sprintf(buffer, "
Invalid position. Please enter a value between 0 and 4095.
");
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            continue;
        }

        // Send command to motor
        // Overwrite last servo mode to positional commands
        writeByte(motorID, 33, 0);

        // Move motor to position
        WritePosEx(motorID, position, speed, acceleration);

        // Provide feedback
        sprintf(buffer, "
Motor %d moved to position %d.
", motorID, position);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

        // Wait for movement to complete
        HAL_Delay(1000); // Adjust delay as needed
    }
}
void loop()
{

  // No longer need to scan the assigned motorID, after writing/hardcoding new ID to EEPROM
  //scanMotors();

  Motor_ID_Horizontal = Ping(1);
  Motor_ID_Vertical = Ping(2);
  char buffer[100]; // Buffer to hold the string to transmit
  if((Motor_ID_Horizontal!=-1) && (Motor_ID_Vertical!=-1)){
	  // Format the message with ID and transmit
	sprintf(buffer, "
Servo ID at X: %d, Servo ID at Y: %d
", Motor_ID_Horizontal, Motor_ID_Vertical);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//    printf("Servo ID:%d
", ID);
    //HAL_Delay(100);
  }

  else{
	sprintf(buffer, "
Ping servo ID error! Motor ID at X: %d, Motor ID at Y: %d
", Motor_ID_Horizontal, Motor_ID_Vertical);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//    printf("Ping servo ID error!
");
    //HAL_Delay(100);
  }

}

void scanMotors() {
    char buffer[255];
    int motorIDs[255];
    int motorCount = 0;
    for (int id = 0; id <= 255; id++) {
        int result = Ping(id);
        if (result != -1) {
            // Motor with ID 'id' is present
            motorIDs[motorCount++] = id;
            sprintf(buffer, "Found motor with ID: %d
", id);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        }
        HAL_Delay(10); // Small delay between pings
    }

    if (motorCount == 0) {
        sprintf(buffer, "No motors found on the bus.
");
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    } else {
        sprintf(buffer, "Total motors found: %d
", motorCount);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

        // Now assign motors to variables
        if (motorCount >= 1) {
            Motor_ID_Horizontal = motorIDs[0];
            sprintf(buffer, "Assigned Motor_ID_Horizontal = %d
", Motor_ID_Horizontal);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        }
        if (motorCount >= 2) {
            Motor_ID_Vertical = motorIDs[1];
            sprintf(buffer, "Assigned Motor_ID_Vertical = %d
", Motor_ID_Vertical);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        }
    }
}


void displayRegisterData(void)
{

  int Pos;
  int Speed;
  int Load;
  int Voltage;
  int Temper;
  int Move;
  int Current;

  // Check feedback
  if(FeedBack(1) != -1){
    Pos = ReadPos(-1);
    Speed = ReadSpeed(-1);
    Load = ReadLoad(-1);
    Voltage = ReadVoltage(-1);
    Temper = ReadTemper(-1);
    Move = ReadMove(-1);
    Current = ReadCurrent(-1);

    // Transmit values using HAL_UART_Transmit
    sprintf(buffer, "Pos: %d
", Pos);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Speed: %d
", Speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Load: %d
", Load);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Voltage: %d
", Voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Temper: %d
", Temper);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Move: %d
", Move);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Current: %d
", Current);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    HAL_Delay(10);
  } else {
    sprintf(buffer, "FeedBack err
");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(2000);
  }

  // Read and transmit servo position
  Pos = ReadPos(1);
  if(Pos != -1){
    sprintf(buffer, "Servo position: %d
", Pos);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read position err
");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo voltage
  Voltage = ReadVoltage(1);
  if(Voltage != -1){
    sprintf(buffer, "Servo Voltage: %d
", Voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Voltage err
");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo temperature
  Temper = ReadTemper(1);
  if(Temper != -1){
    sprintf(buffer, "Servo temperature: %d
", Temper);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read temperature err
");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo speed
  Speed = ReadSpeed(1);
  if(Speed != -1){
    sprintf(buffer, "Servo Speed: %d
", Speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Speed err
");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo load
  Load = ReadLoad(1);
  if(Load != -1){
    sprintf(buffer, "Servo Load: %d
", Load);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Load err
");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo current
  Current = ReadCurrent(1);
  if(Current != -1){
    sprintf(buffer, "Servo Current: %d
", Current);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Current err
");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo move status
  Move = ReadMove(1);
  if(Move != -1){
    sprintf(buffer, "Servo Move: %d
", Move);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Move err
");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Transmit new line
  sprintf(buffer, "
");
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void rotateConstantSpeed(uint8_t ID){

WriteSpe(ID, 2000, 30); // Servo (ID1) rotates at max speed V=2000 steps/sec, acceleration A=50 (50*100 steps/sec²)
  //displayRegisterData();
  HAL_Delay(4000); //5 revolutions
  WriteSpe(ID, 0, 30); // Servo (ID1) stops rotating (V=0) with acceleration A=50 (50*100 steps/sec²)
  //displayRegisterData();
  HAL_Delay(2000);
  WriteSpe(ID, -2000, 30); // Servo (ID1) rotates in reverse at max speed V=-2000 steps/sec, acceleration A=50 (50*100 steps/sec²)
  //displayRegisterData();
  HAL_Delay(1250); // 2000= 2.75 revolutions
  WriteSpe(ID, 0, 30); // Servo (ID1) stops rotating (V=0) with acceleration A=50 (50*100 steps/sec²)
  //displayRegisterData();
  HAL_Delay(2000);
}
float rotateToTargetColumn(uint8_t columnIndex, int targetDetected, int sensorNumber){
	//The sensor datasheet indicates that Column 1 starts from the right, and Column 32 ends on the left.
	//This inverts the column index so that invertedColumnIndex ranges from 31 to 0
	uint8_t invertedColumnIndex = COLUMNS - 1 - columnIndex;
    float MOTOR_MIN_ANGLE;
    float MOTOR_MAX_ANGLE;
    int MOTOR_POSITION_MIN;
    int MOTOR_POSITION_MAX;
    int MOTOR_POSITION_CENTER;

    // Adjust parameters based on sensor number
    if (sensorNumber == SENSOR_MIDDLE) {
        // Middle sensor parameters
        MOTOR_MIN_ANGLE = MOTOR_ANGLE_MIDDLE_MIN;
        MOTOR_MAX_ANGLE = MOTOR_ANGLE_MIDDLE_MAX;
        MOTOR_POSITION_MIN = MOTOR_POSITION_MIDDLE_MIN;
        MOTOR_POSITION_MAX = MOTOR_POSITION_MIDDLE_MAX;
        MOTOR_POSITION_CENTER = MOTOR_POSITION_MIDDLE_CENTER;
    } else if (sensorNumber == SENSOR_LEFT) {
        // Left sensor parameters
        MOTOR_MIN_ANGLE = MOTOR_ANGLE_LEFT_MIN;
        MOTOR_MAX_ANGLE = MOTOR_ANGLE_LEFT_MAX;
        MOTOR_POSITION_MIN = MOTOR_POSITION_LEFT_MIN;
        MOTOR_POSITION_MAX = MOTOR_POSITION_LEFT_MAX;
        MOTOR_POSITION_CENTER = MOTOR_POSITION_LEFT_CENTER;
    } else if (sensorNumber == SENSOR_RIGHT) {
        // Right sensor parameters
        MOTOR_MIN_ANGLE = MOTOR_ANGLE_RIGHT_MIN;
        MOTOR_MAX_ANGLE = MOTOR_ANGLE_RIGHT_MAX;
        MOTOR_POSITION_MIN = MOTOR_POSITION_RIGHT_MIN;
        MOTOR_POSITION_MAX = MOTOR_POSITION_RIGHT_MAX;
        MOTOR_POSITION_CENTER = MOTOR_POSITION_RIGHT_CENTER;
    } else {
        // Default to middle sensor parameters
        MOTOR_MIN_ANGLE = MOTOR_ANGLE_MIDDLE_MIN;
        MOTOR_MAX_ANGLE = MOTOR_ANGLE_MIDDLE_MAX;
        MOTOR_POSITION_MIN = MOTOR_POSITION_MIDDLE_MIN;
        MOTOR_POSITION_MAX = MOTOR_POSITION_MIDDLE_MAX;
        MOTOR_POSITION_CENTER = MOTOR_POSITION_MIDDLE_CENTER;
    }

    uint8_t motorID = Motor_ID_Horizontal;   // Adjust motor ID as needed
    uint16_t speed = 2250; // Set desired speed
    uint8_t acceleration = 40; // Set desired acceleration

    // Overwrite last servo mode, and make it respond to "Positional commands"
    writeByte(motorID, 33, 0);

    float angle;
    int position;

    if (targetDetected == 1) {
        // Map column index to angle over the sensor's field of view
        angle = ((invertedColumnIndex / (float)(COLUMNS - 1)) * (MOTOR_MAX_ANGLE - MOTOR_MIN_ANGLE)) + MOTOR_MIN_ANGLE;

        // Map angle to motor position
        position = (int)(((angle - MOTOR_MIN_ANGLE) * (MOTOR_POSITION_MAX - MOTOR_POSITION_MIN))
                            / (MOTOR_MAX_ANGLE - MOTOR_MIN_ANGLE) + MOTOR_POSITION_MIN);

        // Ensure position is within valid range
        if (position < MOTOR_POSITION_MIN)
            position = MOTOR_POSITION_MIN;
        else if (position > MOTOR_POSITION_MAX)
            position = MOTOR_POSITION_MAX;

    } else {
        // Return to center position when no target is detected
        angle = (MOTOR_MIN_ANGLE + MOTOR_MAX_ANGLE) / 2.0f;
        position = MOTOR_POSITION_CENTER;
    }

    // Move the motor
    WritePosEx(motorID, position, speed, acceleration);

    // Calculate delay based on movement distance
    // Note: At selected speed and acceleration parameters the 360° revolution takes around 2000ms
    uint32_t movementTime = (uint32_t)(fabs(position - MOTOR_POSITION_CENTER)
                            / (float)(MOTOR_POSITION_MAX - MOTOR_POSITION_MIN) * 800);
    HAL_Delay(movementTime);

    // Print debug information
    sprintf(buffer, "Sensor %d: Rotating to column %d, angle %.2f degrees, motor position %d, movement time %lu, targetDetected state %d
",
            sensorNumber, columnIndex, angle, position, movementTime, targetDetected);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    return angle;
}


float rotateToTargetRow(uint8_t rowIndex, int targetDetected){

    const float SENSOR_FOV = 35.0f; // Sensor vertical field of view in degrees

    // Define motor parameters
    const float MOTOR_MIN_ANGLE = -45.0f; // Minimum motor angle in degrees
    const float MOTOR_MAX_ANGLE = 45.0f;  // Maximum motor angle in degrees

    // Calculate motor position range corresponding to -45° to +45°
    const int MOTOR_POSITION_MIN = 1536;  // Corresponds to -45°
    const int MOTOR_POSITION_MAX = 2560;  // Corresponds to +45°

    // Map row index to angle over the sensor's field of view
    float angle = ((rowIndex / (float)(ROWS - 1)) * (MOTOR_MAX_ANGLE - MOTOR_MIN_ANGLE)) + MOTOR_MIN_ANGLE;

    // Map angle to motor position
    int position = (int)(((angle - MOTOR_MIN_ANGLE) * (MOTOR_POSITION_MAX - MOTOR_POSITION_MIN))
                        / (MOTOR_MAX_ANGLE - MOTOR_MIN_ANGLE) + MOTOR_POSITION_MIN);

    // Ensure position is within valid range
    if (position < MOTOR_POSITION_MIN)
        position = MOTOR_POSITION_MIN;
    else if (position > MOTOR_POSITION_MAX)
        position = MOTOR_POSITION_MAX;

    // Move motor to position
    uint8_t motorID = Motor_ID_Vertical;   // Adjust motor ID as needed
    uint16_t speed = 2250; // Set desired speed
    uint8_t acceleration = 40; // Set desired acceleration

    // Overwrite last servo mode, and make it respond to "Positional commands"
    // SMS_STS_MODE corresponds to register #33
    writeByte(motorID, 33, 0);

    // Move the motor
    if (targetDetected == 1) {
        WritePosEx(motorID, position, speed, acceleration);
    } else {
        // Return to center position when no target is detected
    	angle = 0.0f;
    	position = (MOTOR_POSITION_MIN + MOTOR_POSITION_MAX) / 2;
        WritePosEx(motorID, position, speed, acceleration);
    }

    // Calculate delay based on movement distance
    uint32_t movementTime = (uint32_t)(fabs(position - ((MOTOR_POSITION_MIN + MOTOR_POSITION_MAX) / 2))
                            / (float)(MOTOR_POSITION_MAX - MOTOR_POSITION_MIN) * 2000); // Adjust as needed
    HAL_Delay(movementTime);

    // Print debug information
    sprintf(buffer, "Rotating to row %d, angle %.2f degrees, motor position %d, movement time %lu, targetDetected state %d
",
            rowIndex, angle, position, movementTime, targetDetected);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    return angle;
}



