/*
* @Author: Guoxuenan
* @Date:   2016-07-31 17:18:26
* @Last Modified by:   Guoxuenan
* @Last Modified time: 2016-08-08 15:46:01
*/

#include "contiki.h"
#include "platform/mist-exp5438/adc.h"
#include <stdio.h>

#define MAX_NUM 10

void adc_start(void)
{
  //P6.4 ADC option select
  P6SEL |=BIT4;
  //P3.7 输出0. 使能GND
  #if HW_NEW_BIG 
	P6OUT &=~BIT5;  
  #else
    P3OUT &=~BIT7;
  #endif
  
  ADC12CTL0 |=ADC12ON;           //ADC12ON
  ADC12CTL0 |=ADC12ENC;
  __delay_cycles(250);

  ADC12CTL0 |=ADC12SC;      //Sampling and conversion start

}

void adc_close(void)
{

  P6DIR = 0xff;                /*  Direction */
  P6OUT |=0x10 ;  /* P6OUT |=0x10 ; */             //set P6.4 ,output high  this is necessary
  
  #if HW_NEW_BIG 
  	P6OUT |= 0x20;             /* Output */ //set p6.5 output high this is necessary 
  #else
  	P3DIR |= 0xff; //P3DIR |= 0xf0;                /*  Direction */
  	P3OUT |= 0x80; /*P3OUT |= 0x80; */               /* Output */ //set p3.7 output high this is necessary
  #endif
 
  ADC12CTL0 &=~ADC12ENC;       //ADC12 disable

  ADC12CTL0 &=~ADC12ON;        //ADC12 off
 
  while(REFCTL0&REFGENBUSY){};//if ref generator busy,WAIT
  REFCTL0 &=~ REFON;
  
}

void adc_initial(void)
{
 
  #if HW_NEW_BIG 
    P6DIR |=BIT5;
  #else
    P3DIR |=BIT7;
  #endif
  
  #if 1
  //By default,REFMSTR=1 => REFCTL is used to configure the internal reference
  while(REFCTL0&REFGENBUSY){};//if ref generator busy,WAIT
  REFCTL0 |=REFVSEL_1+REFON;//Select internal ref= 2.0V
                                     //Internal Reference ON

  #endif
  
  //ADC12CTL0  |= ADC12REFON+ADC12REF2_5V;
  ADC12CTL0  |=ADC12SHT0_6;   //S&H=128 ADC clks.automatically complete  +ADC12MSC
  ADC12CTL1  |=ADC12SHP+ADC12CSTARTADD2+ADC12CONSEQ_3;      //SAMPCON signal is sourced from the sampling timer
                        //the first conversion is ADC12MEM4.single mode,no repeat

  ADC12MCTL4 |=ADC12INCH_4+ADC12SREF_1;    //A4 ADC input select;Vref and AVss
  
}

uint16_t  get_voltage(void)
{
  static uint8_t count = 0;  
  static uint16_t vcc = 0;

  if(count){
    ++count;
    if(count >=MAX_NUM){
      count=0;
    } 
    return vcc;
  }
  adc_initial();
  adc_start();      
  while (!(ADC12IFG & BIT4)){};
  vcc=ADC12MEM4;
  adc_close();  
  ++count;
  return vcc;
}

