/*
Ping指令测试,测试总线上相应ID舵机是否就绪,广播指令只适用于总线只有一个舵机情况
*/

#include <stdio.h>
#include "stm32f4xx.h"
#include "SCServo.h"
#include "uart.h"
#include <string.h>

extern UART_HandleTypeDef huart2; // For console output
extern UART_HandleTypeDef huart1; // For servo communication

void setup()
{
  //Uart_Init(1000000);

  Uart_Init(115200);
  HAL_Delay(1000);
}

void loop()
{
  int ID = Ping(1);
  char buffer[50]; // Buffer to hold the string to transmit
  if(ID!=-1){
	  // Format the message with ID and transmit
	sprintf(buffer, "\r\nServo ID: %d\r\n", ID);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//    printf("Servo ID:%d\n", ID);
    HAL_Delay(2000);
  }
  else{
	sprintf(buffer, "\r\nPing servo ID error!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//    printf("Ping servo ID error!\n");
    HAL_Delay(2000);
  }
}

void displayRegisterData(void)
{
  char buffer[50]; // Buffer to hold the string to transmit
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
    sprintf(buffer, "Pos: %d\r\n", Pos);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Speed: %d\r\n", Speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Load: %d\r\n", Load);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Voltage: %d\r\n", Voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Temper: %d\r\n", Temper);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Move: %d\r\n", Move);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Current: %d\r\n", Current);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    HAL_Delay(10);
  } else {
    sprintf(buffer, "FeedBack err\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(2000);
  }

  // Read and transmit servo position
  Pos = ReadPos(1);
  if(Pos != -1){
    sprintf(buffer, "Servo position: %d\r\n", Pos);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read position err\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo voltage
  Voltage = ReadVoltage(1);
  if(Voltage != -1){
    sprintf(buffer, "Servo Voltage: %d\r\n", Voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Voltage err\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo temperature
  Temper = ReadTemper(1);
  if(Temper != -1){
    sprintf(buffer, "Servo temperature: %d\r\n", Temper);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read temperature err\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo speed
  Speed = ReadSpeed(1);
  if(Speed != -1){
    sprintf(buffer, "Servo Speed: %d\r\n", Speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Speed err\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo load
  Load = ReadLoad(1);
  if(Load != -1){
    sprintf(buffer, "Servo Load: %d\r\n", Load);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Load err\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo current
  Current = ReadCurrent(1);
  if(Current != -1){
    sprintf(buffer, "Servo Current: %d\r\n", Current);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Current err\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Read and transmit servo move status
  Move = ReadMove(1);
  if(Move != -1){
    sprintf(buffer, "Servo Move: %d\r\n", Move);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(10);
  } else {
    sprintf(buffer, "read Move err\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // Transmit new line
  sprintf(buffer, "\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void rotateConstantSpeed(uint8_t ID){

WriteSpe(ID, 2000, 30); // Servo (ID1) rotates at max speed V=2000 steps/sec, acceleration A=50 (50*100 steps/sec²)
  displayRegisterData();
  HAL_Delay(4000); //5 revolutions
  WriteSpe(ID, 0, 30); // Servo (ID1) stops rotating (V=0) with acceleration A=50 (50*100 steps/sec²)
  displayRegisterData();
  HAL_Delay(2000);
  WriteSpe(ID, -2000, 30); // Servo (ID1) rotates in reverse at max speed V=-2000 steps/sec, acceleration A=50 (50*100 steps/sec²)
  displayRegisterData();
  HAL_Delay(1250); // 2000= 2.75 revolutions
  WriteSpe(ID, 0, 30); // Servo (ID1) stops rotating (V=0) with acceleration A=50 (50*100 steps/sec²)
  displayRegisterData();
  HAL_Delay(2000);
}
