/*
�㲥д����
*/

#include "stm32f4xx.h"
#include "SCServo.h"
#include "uart.h"
#include "wiring.h"

void setup(void)
{
	Uart_Init(115200);
  delay(1000);
}

void loop(void)
{
  WritePosEx(0xfe, 4095, 2250, 50);//���(�㲥),������ٶ�V=2250��/��,���ٶ�A=50(50*100��/��^2),������P1=4095
  delay(2270);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	
  WritePosEx(0xfe, 0, 2250, 50);//���(�㲥),������ٶ�V=2250��/��,���ٶ�A=50(50*100��/��^2),������P1=0
  delay(2270);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
}
