/*
�첽д����
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
  RegWritePosEx(1, 4095, 2250, 50);//���(ID1),������ٶ�V=2250��/��,���ٶ�A=50(50*100��/��^2),������P1=4095
	RegWritePosEx(2, 4095, 2250, 50);//���(ID2),������ٶ�V=2250��/��,���ٶ�A=50(50*100��/��^2),������P1=4095
  RegWriteAction();
	delay(2270);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	
  RegWritePosEx(1, 0, 2250, 50);//���(ID1),������ٶ�V=2250��/��,���ٶ�A=50(50*100��/��^2),������P1=0
	RegWritePosEx(2, 0, 2250, 50);//���(ID2),������ٶ�V=2250��/��,���ٶ�A=50(50*100��/��^2),������P1=0
  RegWriteAction();
	delay(2270);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
}
