/******************************************************************
**===============================================================**
**=================电流检测装置单片机程序=========================**
**可以实现对任意的0-3.3V电压的波形的检测，并输出其电压的峰峰值、频率**
**以及傅里叶分解之后的基波,一次谐波,二次谐波的幅值和频率。其中自带  **
**了绝对值处理电路,并且对利用stm32的dsp处理模块中的fft进行傅里叶分 **
**析.                                                            **
**===============================================================**
**------------------------注意事项-------------------------------**
**该程序在MDK5上编译通过，并且使用ST-Link下载烧录均成功。单片机使用**
**的是stm32的MINNI板（型号是stm32f103RC）,程序运行无误。其中输入引**
**脚为PA1，注意需要将GND引出共地。由于内存和计算量的原因，fft只能使**
**用64个采样点进行操作的函数，并且在数组的处理上无法超过150。在较好**
**的硬件上可以提升这些数值，他们被放置在了宏定义里。这些数值的提升会**
增加频率检测的精度                                                **
**此外，将浮点数输出LCD显示屏上使用的函数是自己写的，仅支持X.XXX的格**
**式，可以根据自己的需要进行修改或者加强                           **
**---------------------------------------------------------------**                    
**=================Final time:2018年7月23日======================**
*******************************************************************/
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "adc.h"
#include "floatLED.h"
#include "math.h"
#include "stm32_dsp.h"
#include "table_fft.h"

#define NPT 64
#define Fs  2500
#define PI2 6.28318530717959
#define NNN 135 /*NNN代表的是采样的时候的采样点的数目*/
#define NUMOF_F 3 /*只得是找寻傅里叶分解中的基波与谐波的数目*/


/********************************************************************
得到幅值的函数，将储存着的实部和虚部分别取出并利用二范数求得平方和
参数说明：傅里叶变换的输出，储存幅值的向量，实部，虚部
这里要注意的是，y代表的是实部，x是虚部
********************************************************************/
void Get_PowerMag(long* OutArray, unsigned long* MagArray,u16* y,u16* x)
{
    signed short lX,lY;
    double X,Y;
	float Mag;
    unsigned short i;
    for(i=0; i<NPT/2; i++)
    {
        lX  = (OutArray[i] << 16) >> 16;
        lY  = (OutArray[i] >> 16);
        x[i]=lX;y[i]=lY;
		X = ((float)lX/32768*NPT);
        Y = ((float)lY/32768*NPT);
        Mag = sqrt(X * X + Y * Y)/NPT ;
        if(i == 0)
          MagArray[i] = (unsigned long)(Mag * 32768);
       else
           MagArray[i] = (unsigned long)(Mag * 65535);
    }
}

/***********************************************************************
将电平的输入信号转化为有正有负的周期信号的函数
其中
输入序列的长度是固定的，NNN
输出序列的长度必须是进行FFT所需求的长度，即 NPT。其中输出的序列已经按照实部高
16位虚部低16位的原则进行变换了。
************************************************************************/
void ADC2Nomal(int* before,long* after)
{
	u16 i,j,k;
	u16 Find_zero=0;
	i=1;//i为1时表示不取反，i为0时表示取反。
	k=0;//用来作为输出序列的索引
	for(j=0;j<NNN;j++)//这一层循环是用来赋值的
	{
		if(k>=NPT)
		{
			break;
		}
		
		if((before[j]<0.001)&&(Find_zero==0))
		{
			Find_zero=1;
			if(i==1)
				i=0;
			else
				i=1;
		}
	
		else if (before[j]>0.001)
		{
			Find_zero=0;
			if(i==1)
			{
				after[k]=((signed short)before[j])<<16;
			}
			else if(i==0)
			{
				after[k]=((signed short)-before[j])<<16;
			}
			k++;
		}
			
	}
}

/**********************************************************
功能：找寻非正弦波中最大的三种正弦波的频率和幅值
他们分别对应着的是：基波，一次谐波，二次谐波
===================这一部分有一些问题======================
(问题主要集中在：为什么上来的第一个频率是几千Hz？？？)
**********************************************************/
void find_frequency(unsigned long* MagArray,u16* Frequency)
{
	u16 Index[NPT];
	unsigned long cc;
	double per_frequency;
	u16 i,j,c;
	/*对index进行赋值*/
	for(i=0;i<NPT;i++)
	{
		Index[i]=i;
	}
	/*排序算法的实现，保证前几位是最大的*/
	for(i=0;i<NPT;i++)
	{
		for(j=0;j<NPT-i;j++)
		{
			if(MagArray[j]<MagArray[j+1])
			{
				/*对序列进行排序，保证整体排序的正确性*/
				cc=MagArray[j];
				MagArray[j]=MagArray[j+1];
				MagArray[j+1]=cc;
				/*对索引进行排序，以方便得到频率*/
				c=Index[j];
				Index[j]=Index[j+1];
				Index[j+1]=c;
			}
		}
	}
	per_frequency=(double)Fs/NPT;
	for(i=0;i<NUMOF_F;i++)
	{
		Frequency[i]=(u16)(Index[i+1]*per_frequency);
		
	}
}

/*********************************************
使用此函数来尝试找到周期信号的幅值
max代表信号中的最大值，
shulie代表的是AD转换之后所得到数字序列，长度为NNN
返回值即为频率
**********************************************/
u16 only_find_frequency(unsigned long* MagArray, u16* Frequency)
{
	/*
	u16 cc,i,num;
	cc=0;//只有在cc为1时采取找寻频率
	num=0;//表示从最大值开始所经过的点数
	for(i=0;i<NNN;i++)
	{
		
		if((double)(fabs(shulie[i]*3.3/4096)-0.7*max)<0.1)
		{
			if(cc==2)
			{
				break;
			}
			if(cc==1)
			{
				cc=2;
			}
			cc=1;
		}
		else
		{
			num++;
		}
	}
	return (2500/num);
	*/
	double bili;
	bili=(float)MagArray[0]/(float)(MagArray[1]+MagArray[0]);
	return (u16)((float)Frequency[0]*bili+(float)Frequency[1]*(1-bili));
	
}

int main(void)
{
	/*计数采样部分*/
	u16 count;/*设计一个计数器，用来计划采集足够高的数*/
	double max_Vcc;/*设计这个东西来保存其最大值*/
	int VCC_shu_lie[NNN];/*设计这个东西用来保存采样得到的100个点数列*/
	long InArray[NPT];/*进行正负交替处理后的信号，相当于FFT的输入*/
	long OutArray[NPT];/*FFT的输出*/
	unsigned long MagArray[NPT];/*FFT的幅值的输出*/
	u16 x[NPT];/*实部组成的向量*/
	u16 y[NPT];/*虚部组成的向量*/
	//signed short adcx;/*一个中间量，相当于读取了ADC后的数字*/
	u16 Frequency[NUMOF_F];/*数组用来存储波的频率*/
	//float temp;
	u16 cc;
	/*计算频率部分*/
	//float T;
	//u16 T0,f;
	u16 ccc;
	/*初始化部分*/
	delay_init();	    	 //延时函数初始化	  
	uart_init(9600);	 	//串口初始化为9600
	LED_Init();		  		//初始化与LED连接的硬件接口
 	LCD_Init();
 	Adc_Init();		  		//ADC初始化	    
	count=0;
	max_Vcc=0;
	ccc=1;
	/*******************************************************************
	=========================字体显示部分===============================
	包括三部分：
	（1）显示队名，显示成员名称，显示时间，显示所要实现的功能。
	（2）显示正弦信号时的峰峰值和频率
	（3）显示非正弦信号的基波的峰峰值和频率
	*******************************************************************/
	
	
	/******************************************************************/
	POINT_COLOR=GREEN;
	LCD_ShowString(0,0,400,16,16,"==============================");
	POINT_COLOR=RED;
	LCD_ShowString(0,20,400,16,16,"Electric Current Sensor");
	LCD_ShowString(0,40,200,16,16,"END TIME: 2018,7,23");	
	//POINT_COLOR=BLUE;
	/******************************************************************
	显示正弦信号（LED0亮起）
	包括：峰峰值，频率
	*******************************************************************/
	POINT_COLOR=GREEN;
	LCD_ShowString(0,60,400,16,16,"========Only Ok in SIN========");	      
	POINT_COLOR=BLUE;
	LCD_ShowString(0,80,200,16,16,"IPP:0.000A");	    
	LCD_ShowString(0,100,200,16,16,"Fre:0000Hz");
	/******************************************************************
	显示非正弦信号（LED1亮起）
	包括：基波峰峰值、频率，一次谐波峰峰值、频率，二次谐波峰峰值、频率
	******************************************************************/
	POINT_COLOR=GREEN;
	LCD_ShowString(0,120,400,16,16,"======Only Ok in NOR SIN======");	      
	POINT_COLOR=BLUE;
	LCD_ShowString(0,140,200,16,16,">>>Fundermental wave:");
	LCD_ShowString(0,160,200,16,16,"AMP:0.000A");
	LCD_ShowString(0,180,200,16,16,"Fre:0000Hz");
	LCD_ShowString(0,200,200,16,16,">>>Primary harmonic wave:");	
	LCD_ShowString(0,220,200,16,16,"AMP:0.000A");
	LCD_ShowString(0,240,200,16,16,"Fre:0000Hz");
	LCD_ShowString(0,260,200,16,16,">>>second harmonic wave:");
	LCD_ShowString(0,280,200,16,16,"AMP:0.000A");
	LCD_ShowString(0,300,200,16,16,"Fre:0000Hz");
	
	while(1)
	{
		if(count<200)
		{
			/*捕捉AD转换之后的信号（未转换）*/
			LED0=!LED0;
			VCC_shu_lie[count]=Get_Adc(ADC_Channel_1);//捕捉的是239.5的位置
			
			//temp=(float)adcx*(3.3/4096);
			
			/*尝试求得最大值*/
			//LCD_ShowxNum(60,130,adcx,4,16,0);
			if((float)VCC_shu_lie[count]*(3.3/4096)>max_Vcc)
			{
				max_Vcc=(float)VCC_shu_lie[count]*3.3/4096;		
			}
			//VCC_shu_lie[count]=temp;
			delay_us(400-18);//这一行非常重要！！！！！
		}
		
		else
		{
			/*先将信号进行转换*/
			ADC2Nomal(VCC_shu_lie,InArray);
						
			//LCD_ShowxNum(156,170,f,4,16,0);
			//Print_Float_LCD(180,150,max_Vcc,16);
			
			if(ccc==1)
			{	
				cr4_fft_64_stm32(OutArray,InArray,NPT);
				Get_PowerMag(OutArray,MagArray,x,y);	
				find_frequency(MagArray,Frequency);
				
				for(cc=0;cc<NUMOF_F;cc++)
				{
					LCD_ShowxNum(32,180+(60*cc),Frequency[cc],4,16,0);
					//LCD_ShowxNum(40,0+(20*cc),MagArray[cc],6,16,0);
					Print_Float_LCD(32,160+(60*cc),(((double)MagArray[cc]*3.3/4096*1.11545)+0.00719),16);
					if(cc==0)
					{
						Print_Float_LCD(32,160,max_Vcc-0.011,16);
					}
				}
				/*显示总体的频率*/
				LCD_ShowxNum(32,100,only_find_frequency(MagArray,Frequency),4,16,0);
				/*显示峰峰值*/
				Print_Float_LCD(32,80,2*max_Vcc*1.11545+0.00719,16);
				ccc=0;
			}
				delay_ms(250);
				//LED0=!LED0;	
		}	
		count++;
	}											    
}	

