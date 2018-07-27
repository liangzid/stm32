/******************************************************************
**===============================================================**
**=================�������װ�õ�Ƭ������=========================**
**����ʵ�ֶ������0-3.3V��ѹ�Ĳ��εļ�⣬��������ѹ�ķ��ֵ��Ƶ��**
**�Լ�����Ҷ�ֽ�֮��Ļ���,һ��г��,����г���ķ�ֵ��Ƶ�ʡ������Դ�  **
**�˾���ֵ�����·,���Ҷ�����stm32��dsp����ģ���е�fft���и���Ҷ�� **
**��.                                                            **
**===============================================================**
**------------------------ע������-------------------------------**
**�ó�����MDK5�ϱ���ͨ��������ʹ��ST-Link������¼���ɹ�����Ƭ��ʹ��**
**����stm32��MINNI�壨�ͺ���stm32f103RC��,����������������������**
**��ΪPA1��ע����Ҫ��GND�������ء������ڴ�ͼ�������ԭ��fftֻ��ʹ**
**��64����������в����ĺ���������������Ĵ������޷�����150���ڽϺ�**
**��Ӳ���Ͽ���������Щ��ֵ�����Ǳ��������˺궨�����Щ��ֵ��������**
����Ƶ�ʼ��ľ���                                                **
**���⣬�����������LCD��ʾ����ʹ�õĺ������Լ�д�ģ���֧��X.XXX�ĸ�**
**ʽ�����Ը����Լ�����Ҫ�����޸Ļ��߼�ǿ                           **
**---------------------------------------------------------------**                    
**=================Final time:2018��7��23��======================**
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
#define NNN 135 /*NNN������ǲ�����ʱ��Ĳ��������Ŀ*/
#define NUMOF_F 3 /*ֻ������Ѱ����Ҷ�ֽ��еĻ�����г������Ŀ*/


/********************************************************************
�õ���ֵ�ĺ������������ŵ�ʵ�����鲿�ֱ�ȡ�������ö��������ƽ����
����˵��������Ҷ�任������������ֵ��������ʵ�����鲿
����Ҫע����ǣ�y�������ʵ����x���鲿
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
����ƽ�������ź�ת��Ϊ�����и��������źŵĺ���
����
�������еĳ����ǹ̶��ģ�NNN
������еĳ��ȱ����ǽ���FFT������ĳ��ȣ��� NPT����������������Ѿ�����ʵ����
16λ�鲿��16λ��ԭ����б任�ˡ�
************************************************************************/
void ADC2Nomal(int* before,long* after)
{
	u16 i,j,k;
	u16 Find_zero=0;
	i=1;//iΪ1ʱ��ʾ��ȡ����iΪ0ʱ��ʾȡ����
	k=0;//������Ϊ������е�����
	for(j=0;j<NNN;j++)//��һ��ѭ����������ֵ��
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
���ܣ���Ѱ�����Ҳ��������������Ҳ���Ƶ�ʺͷ�ֵ
���Ƿֱ��Ӧ�ŵ��ǣ�������һ��г��������г��
===================��һ������һЩ����======================
(������Ҫ�����ڣ�Ϊʲô�����ĵ�һ��Ƶ���Ǽ�ǧHz������)
**********************************************************/
void find_frequency(unsigned long* MagArray,u16* Frequency)
{
	u16 Index[NPT];
	unsigned long cc;
	double per_frequency;
	u16 i,j,c;
	/*��index���и�ֵ*/
	for(i=0;i<NPT;i++)
	{
		Index[i]=i;
	}
	/*�����㷨��ʵ�֣���֤ǰ��λ������*/
	for(i=0;i<NPT;i++)
	{
		for(j=0;j<NPT-i;j++)
		{
			if(MagArray[j]<MagArray[j+1])
			{
				/*�����н������򣬱�֤�����������ȷ��*/
				cc=MagArray[j];
				MagArray[j]=MagArray[j+1];
				MagArray[j+1]=cc;
				/*���������������Է���õ�Ƶ��*/
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
ʹ�ô˺����������ҵ������źŵķ�ֵ
max�����ź��е����ֵ��
shulie�������ADת��֮�����õ��������У�����ΪNNN
����ֵ��ΪƵ��
**********************************************/
u16 only_find_frequency(unsigned long* MagArray, u16* Frequency)
{
	/*
	u16 cc,i,num;
	cc=0;//ֻ����ccΪ1ʱ��ȡ��ѰƵ��
	num=0;//��ʾ�����ֵ��ʼ�������ĵ���
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
	/*������������*/
	u16 count;/*���һ���������������ƻ��ɼ��㹻�ߵ���*/
	double max_Vcc;/*���������������������ֵ*/
	int VCC_shu_lie[NNN];/*����������������������õ���100��������*/
	long InArray[NPT];/*�����������洦�����źţ��൱��FFT������*/
	long OutArray[NPT];/*FFT�����*/
	unsigned long MagArray[NPT];/*FFT�ķ�ֵ�����*/
	u16 x[NPT];/*ʵ����ɵ�����*/
	u16 y[NPT];/*�鲿��ɵ�����*/
	//signed short adcx;/*һ���м������൱�ڶ�ȡ��ADC�������*/
	u16 Frequency[NUMOF_F];/*���������洢����Ƶ��*/
	//float temp;
	u16 cc;
	/*����Ƶ�ʲ���*/
	//float T;
	//u16 T0,f;
	u16 ccc;
	/*��ʼ������*/
	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(9600);	 	//���ڳ�ʼ��Ϊ9600
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
 	LCD_Init();
 	Adc_Init();		  		//ADC��ʼ��	    
	count=0;
	max_Vcc=0;
	ccc=1;
	/*******************************************************************
	=========================������ʾ����===============================
	���������֣�
	��1����ʾ��������ʾ��Ա���ƣ���ʾʱ�䣬��ʾ��Ҫʵ�ֵĹ��ܡ�
	��2����ʾ�����ź�ʱ�ķ��ֵ��Ƶ��
	��3����ʾ�������źŵĻ����ķ��ֵ��Ƶ��
	*******************************************************************/
	
	
	/******************************************************************/
	POINT_COLOR=GREEN;
	LCD_ShowString(0,0,400,16,16,"==============================");
	POINT_COLOR=RED;
	LCD_ShowString(0,20,400,16,16,"Electric Current Sensor");
	LCD_ShowString(0,40,200,16,16,"END TIME: 2018,7,23");	
	//POINT_COLOR=BLUE;
	/******************************************************************
	��ʾ�����źţ�LED0����
	���������ֵ��Ƶ��
	*******************************************************************/
	POINT_COLOR=GREEN;
	LCD_ShowString(0,60,400,16,16,"========Only Ok in SIN========");	      
	POINT_COLOR=BLUE;
	LCD_ShowString(0,80,200,16,16,"IPP:0.000A");	    
	LCD_ShowString(0,100,200,16,16,"Fre:0000Hz");
	/******************************************************************
	��ʾ�������źţ�LED1����
	�������������ֵ��Ƶ�ʣ�һ��г�����ֵ��Ƶ�ʣ�����г�����ֵ��Ƶ��
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
			/*��׽ADת��֮����źţ�δת����*/
			LED0=!LED0;
			VCC_shu_lie[count]=Get_Adc(ADC_Channel_1);//��׽����239.5��λ��
			
			//temp=(float)adcx*(3.3/4096);
			
			/*����������ֵ*/
			//LCD_ShowxNum(60,130,adcx,4,16,0);
			if((float)VCC_shu_lie[count]*(3.3/4096)>max_Vcc)
			{
				max_Vcc=(float)VCC_shu_lie[count]*3.3/4096;		
			}
			//VCC_shu_lie[count]=temp;
			delay_us(400-18);//��һ�зǳ���Ҫ����������
		}
		
		else
		{
			/*�Ƚ��źŽ���ת��*/
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
				/*��ʾ�����Ƶ��*/
				LCD_ShowxNum(32,100,only_find_frequency(MagArray,Frequency),4,16,0);
				/*��ʾ���ֵ*/
				Print_Float_LCD(32,80,2*max_Vcc*1.11545+0.00719,16);
				ccc=0;
			}
				delay_ms(250);
				//LED0=!LED0;	
		}	
		count++;
	}											    
}	

