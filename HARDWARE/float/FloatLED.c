#include "stdio.h"
#include "FloatLED.h"
#include "lcd.h"                 // Device header


void Print_Float_LCD(u16 x,u16 y,float float_num,u8 size)
{
	u16 temp;
	temp=float_num;
	LCD_ShowxNum(x,y,temp,1,size,0);
	float_num=float_num-temp;
	float_num*=1000;
	LCD_ShowxNum(x+16,y,float_num,3,16,0x80);
}
