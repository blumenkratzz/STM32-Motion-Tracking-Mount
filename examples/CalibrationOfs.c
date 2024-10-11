/*
中位校准例子
*/

#include "stm32f4xx.h"
#include "SCServo.h"
#include "uart.h"
#include "wiring.h"

void setup()
{
	Uart_Init(115200);
  delay(1000);
}

void loop()
{
  CalibrationOfs(1);
  while(1);
}
