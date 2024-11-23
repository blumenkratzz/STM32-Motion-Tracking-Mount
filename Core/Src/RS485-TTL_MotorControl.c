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
float rotateToTargetColumn(uint8_t columnIndex, int targetDetected){

	 const int COLUMNS = 32;
	    const float SENSOR_FOV = 55.0f; // Sensor field of view in degrees
	    const float anglePerColumn = SENSOR_FOV / COLUMNS;


	    // Define motor parameters
	    const float MOTOR_MIN_ANGLE = -180.0f; // Minimum motor angle in degrees
	    const float MOTOR_MAX_ANGLE = 180.0f;  // Maximum motor angle in degrees
	    const int MOTOR_POSITION_MIN = 0;      // Minimum motor position
	    const int MOTOR_POSITION_MAX = 4095;   // Maximum motor position

	    float angle = ((columnIndex / (float)(COLUMNS - 1)) * (MOTOR_MAX_ANGLE - MOTOR_MIN_ANGLE)) + MOTOR_MIN_ANGLE;
	    // Map angle to motor position
	    int position = (int)(((angle - MOTOR_MIN_ANGLE) * (MOTOR_POSITION_MAX - MOTOR_POSITION_MIN)) / (MOTOR_MAX_ANGLE - MOTOR_MIN_ANGLE) + MOTOR_POSITION_MIN);

	    // Ensure position is within valid range
	    if (position < MOTOR_POSITION_MIN)
	        position = MOTOR_POSITION_MIN;
	    else if (position > MOTOR_POSITION_MAX)
	        position = MOTOR_POSITION_MAX;

	    // Move motor to position
	    uint8_t motorID = Motor_ID_Horizontal;   // Adjust motor ID as needed
	    uint16_t speed = 2250; // Set desired speed
	    uint8_t acceleration = 40; // Set desired acceleration

	    // Print debug information
	    sprintf(buffer, "Rotating to column %d, angle %.2f degrees, motor position %d
", columnIndex, angle, position);
	    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

		// Overwrite last servo mode, and make it respond to "Positional commands"
		// SMS_STS_MODE Corresponds to register # 33
		writeByte(motorID, 33, 0);
		//if (targetDetected==1){
	    WritePosEx(motorID, position, speed, acceleration);
		//}
	    // Calculate delay based on movement distance
	    // Assuming a linear relationship for simplicity
	    uint32_t movementTime = (uint32_t)(fabs(position - MOTOR_POSITION_MAX / 2) / (float)MOTOR_POSITION_MAX * 2000); // Adjust the multiplier as needed
	    //HAL_Delay(movementTime);

	//WritePosEx(1, 4095, 2250, 30); // Servo motor (ID1), with maximum speed V=2250 steps/sec, acceleration A=50 (50*100 steps/sec²), moves to position P1=4095
	//HAL_Delay(2270); // Delay calculation: [(P1-P0)/V]*1000 + [V/(A*100)]*1000

  	//WritePosEx(1, 0, 2250, 30); // Servo motor (ID1), with maximum speed V=2250 steps/sec, acceleration A=50 (50*100 steps/sec²), moves to position P1=0
  	//HAL_Delay(2270); // Delay calculation: [(P1-P0)/V]*1000 + [V/(A*100)]*1000
  	return angle;
}

float rotateToTargetRow(uint8_t rowIndex, int targetDetected){
    const int ROWS = 24;
    const float SENSOR_FOV = 35.0f; // Sensor vertical field of view in degrees

    // Define motor parameters
    const float MOTOR_MIN_ANGLE = -180.0f; // Minimum motor angle in degrees
    const float MOTOR_MAX_ANGLE = 180.0f;  // Maximum motor angle in degrees
    const int MOTOR_POSITION_MIN = 0;      // Minimum motor position
    const int MOTOR_POSITION_MAX = 4095;   // Maximum motor position

    // Map row index to angle over the full motor rotation range
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
    uint8_t motorID = Motor_ID_Vertical;   // Adjust motor ID as needed (assuming a different motor for vertical movement)
    uint16_t speed = 2250; // Set desired speed
    uint8_t acceleration = 40; // Set desired acceleration

    // Print debug information
    sprintf(buffer, "Rotating to row %d, angle %.2f degrees, motor position %d
", rowIndex, angle, position);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);


	// Overwrite last servo mode, and make it respond to "Positional commands"
	// SMS_STS_MODE Corresponds to register # 33
	writeByte(motorID, 33, 0);
    // Move the motor if target is detected
    if (targetDetected == 1) {
        WritePosEx(motorID, position, speed, acceleration);
    }

    // Calculate delay based on movement distance
    uint32_t movementTime = (uint32_t)(fabs(position - (MOTOR_POSITION_MAX / 2))
                            / (float)MOTOR_POSITION_MAX * 2000); // Adjust as needed
    HAL_Delay(1000);

    return angle;
}


