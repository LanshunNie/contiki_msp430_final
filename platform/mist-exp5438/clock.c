/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

#include "contiki.h"
#include "sys/energest.h"
#include "sys/clock.h"
#include "sys/etimer.h"
#include "rtimer-arch.h"
#include "dev/watchdog.h"
#include "isr_compat.h"
#include "sys/rtimer.h"

#include "dev/leds.h"
 
#include "contikimac.h"
#include "node-id.h"

#include "auto-sleep.h"

#if CONTIKI_CONF_NETSYNCH
#include "netsynch.h"
#include "task-schedule.h"
#endif
//zhangwei set changed for load balance
#if  WITH_ENERGY_EFFICIENCY
#include "rdc-efficiency.h"
#endif

#if LOW_LATENCY
#include "low-latency.h"

#if ROOTNODE
#include "low-latency-msg.h" 
#endif
#endif

static uint8_t low_latency_flag=0;

#define INTERVAL (RTIMER_ARCH_SECOND / CLOCK_SECOND)

#define MAX_TICKS (~((clock_time_t)0) / 2)

#define CLOCK_LT(a, b) ((int16_t)((a)-(b)) < 0)
static volatile unsigned long seconds;

static volatile clock_time_t count = 0;

#if TRXEB1120_CONF_LOWPOWER
static volatile uint8_t  active_flag_one_second_before = 0;
static volatile uint16_t  init_net_flag = 0;

static volatile uint8_t  active_flag = 1;

static uint8_t schedule_bit[18]; 
static soft_time  timenow;

static int cal_offest = 0;            
static uint32_t cal_interval = 0;
static uint32_t cal_count = 0;

static uint16_t days=0;
static uint8_t  ledon_flag =0;

static void update_soft_time();

#endif
/* last_tar is used for calculating clock_fine */
static volatile uint16_t last_tar = 0;

//zhangwei set changed for load balance
static volatile unsigned long rtc_second;
/*---------------------------------------------------------------------------*/
static inline uint16_t
read_tar(void)
{
  /* Same as clock_counter(), but can be inlined */
  uint16_t t1, t2;
  do {
    t1 = TA1R;
    t2 = TA1R;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
ISR(TIMER1_A1, timera1)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  /* watchdog_start(); */

  if(TA1IV == 2) {

    /* HW timer bug fix: Interrupt handler called before TR==CCR.
     * Occurs when timer state is toggled between STOP and CONT. */
    while(TA1CTL & MC1 && TA1CCR1 - TA1R == 1);

    last_tar = read_tar();
    /* Make sure interrupt time is future */
    while(!CLOCK_LT(last_tar, TA1CCR1)) {
      TA1CCR1 += INTERVAL;
      ++count;

      /* Make sure the CLOCK_CONF_SECOND is a power of two, to ensure
	 that the modulo operation below becomes a logical and and not
	 an expensive divide. Algorithm from Wikipedia:
	 http://en.wikipedia.org/wiki/Power_of_two */
#if (CLOCK_CONF_SECOND & (CLOCK_CONF_SECOND - 1)) != 0
#error CLOCK_CONF_SECOND must be a power of two (i.e., 1, 2, 4, 8, 16, 32, 64, ...).
#error Change CLOCK_CONF_SECOND in contiki-conf.h.
#endif
      if(count % CLOCK_CONF_SECOND == 0) {
    	++seconds;
        energest_flush();
      }
      last_tar = read_tar();
    }

    if(etimer_pending() &&
       (etimer_next_expiration_time() - count - 1) > MAX_TICKS) {
      etimer_request_poll();
      LPM4_EXIT;
    }

  }
  /*  if(process_nevents() >= 0) {
    LPM4_EXIT;
    }*/

  /* watchdog_stop(); */

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  clock_time_t t1, t2;
  do {
    t1 = count;
    t2 = count;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
void
clock_set(clock_time_t clock, clock_time_t fclock)
{
  TA1R = fclock;
  TA1CCR1 = fclock + INTERVAL;
  count = clock;
}
/*---------------------------------------------------------------------------*/
int
clock_fine_max(void)
{
  return INTERVAL;
}
/*---------------------------------------------------------------------------*/
unsigned short
clock_fine(void)
{
  unsigned short t;
  /* Assign last_tar to local varible that can not be changed by interrupt */
  t = last_tar;
  /* perform calc based on t, TAR will not be changed during interrupt */
  return (unsigned short) (TA1R - t);
}
/*---------------------------------------------------------------------------*/
void
clock_init(void)
{
  dint();

  /* Select SMCLK (2.4576MHz), clear TAR */
  /* TACTL = TASSEL1 | TACLR | ID_3; */

  /* Select ACLK 32768Hz clock, divide by 2 */
  /*  TACTL = TASSEL0 | TACLR | ID_1;*/

  /* Select ACLK 32768Hz clock */
  /* TACTL = TASSEL0 | TACLR; */

#if INTERVAL==32768/CLOCK_SECOND
  TA1CTL = TASSEL0 | TACLR;
#elif INTERVAL==16384/CLOCK_SECOND
  TA1CTL = TASSEL0 | TACLR | ID_1;
#else
#error NEED TO UPDATE clock.c to match interval!
#endif

  /* Initialize ccr1 to create the X ms interval. */
  /* CCR1 interrupt enabled, interrupt occurs when timer equals CCR. */
  TA1CCTL1 = CCIE;

  /* Interrupt after X ms. */
  TA1CCR1 = INTERVAL;

  /* Start Timer_A in continuous mode. */
  TA1CTL |= MC1;

  count = 0;

  /* Enable interrupts. */
  eint();

}
/*---------------------------------------------------------------------------*/
/**
 * Delay the CPU for a multiple of 2.83 us.
 */
void
clock_delay(unsigned int i)
{
  while(i--) {
    _NOP();
  }
}
/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 10 ms.
 *
 */
void
clock_wait(clock_time_t i)
{
  clock_time_t start;

  start = clock_time();
  while(clock_time() - start < (clock_time_t)i);
}
/*---------------------------------------------------------------------------*/
void
clock_set_seconds(unsigned long sec)
{
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
  unsigned long t1, t2;
  do {
    t1 = seconds;
    t2 = seconds;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
clock_counter(void)
{
  rtimer_clock_t t1, t2;
  do {
    t1 = TA1R;
    t2 = TA1R;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
#if TRXEB1120_CONF_LOWPOWER
void
rtc_init (void)
{
  
  dint ();
   
  // timenow={0,19,30};
  timenow.hour  =0;
  timenow.minute=0;
#if ROOTNODE
  timenow.minute=9;
#endif
  timenow.sec   =0;
 // interval 1s
  RTCCTL01  = RTCTEVIE + RTCSSEL_2 + RTCTEV_0; // Counter Mode, RTC1PS, 8-bit ovf
  RTCCTL01 &=~ RTCTEVIFG;                                          // overflow interrupt enable
  RTCPS0CTL =             RT0PSDIV_2;         // ACLK, /8, start timer
  RTCPS1CTL = RT1SSEL_2 + RT1PSDIV_3;         // out from RT0PS, /128, start timer 

/*
  RTCCTL01 |= RTCAIE  + RTCBCD + RTCHOLD + RTCMODE+RTCRDYIE;
                                            // RTC enable, BCD mode, RTC hold
                                            // enable RTC read ready interrupt
                                            // enable RTC time event interrupt
  RTCYEAR = 0x2016;                         // Year = 0x2010
  RTCMON = 0x3;                             // Month = 0x04 = April
  RTCDAY = 0x4;                            // Day = 0x05 = 5th
  RTCDOW = 0x5;                            // Day of week = 0x01 = Monday
  RTCHOUR = 0x00;                           // Hour = 0x10
  RTCMIN =  0x00;                            // Minute = 0x32
  RTCSEC =  0x00;                            // Seconds = 0x45

  // RTCADOWDAY =                         // RTC Day of week alarm 
  // RTCADAY    =                           // RTC Day Alarm 
  // RTCAHOUR   =                        // RTC Hour Alarm
  //RTCAMIN = (((RTCMIN & 0x70)+0x10)% 0x60)|0x80;                       // RTC Minute Alarm

  RTCCTL01 &= ~(RTCHOLD);                   // Start RTC calendar mode
  
  */
  // printf("init rtc time %d:%d:%d;\n",timenow.hour,timenow.minute,timenow.sec );
  eint (); 
}

static void 
update_soft_time()
{
  timenow.sec+=1;
  if( timenow.sec/60){ 
      ++timenow.minute;
      timenow.sec=0;
  }

  if( timenow.minute/60 ){
      ++timenow.hour; 
      timenow.minute=0;
  }
  if( timenow.hour/24 ){
      timenow.hour=0;
      ++days;
  }
}

ISR(RTC,rtcisr)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  ENERGEST_OFF(ENERGEST_TYPE_LPM);
  watchdog_start();
  
  switch(__even_in_range(RTCIV,16)){
    case RTC_NONE:                          // No interrupts
      break;

    case RTC_RTCRDYIFG:                     // RTCRDYIFG
      break;

    case RTC_RTCTEVIFG:                     // RTCEVIFG

// printf("now_sec:%u\n",get_rtimer_seconds());

#if ROOTNODE
    update_soft_time();
#else      
    // if(ledon_flag){
    //   leds_off(LEDS_ALL);
    //   ledon_flag =0;
    // }

    // ++cal_count;
    // if(cal_offest != 0 && cal_count >= cal_interval){
    //   cal_count = 0;
    //   if(cal_offest < 0){
    //     update_soft_time();
    //     update_soft_time();
    //   }
    // }else{
      update_soft_time();
   // }
#endif

       active_flag_one_second_before=active_flag;
       set_active_flag(timenow.hour,timenow.minute,timenow.sec);

       #if LOW_LATENCY
       low_latency_flag= set_lowLatency_flag(timenow);
       #endif
	   
      if(active_flag!=active_flag_one_second_before){
         // set_cycletime(active_flag);
         if(active_flag ==1){
           // P3OUT &= ~0x40; 
           task_schedule_change();

       #if LOW_LATENCY & ROOTNODE
           low_latency_msg_send();
       #endif

           //zhangwei set changed for load balance
           rtc_second = 0;  //zhangwei set changed for energy efficiency

           #if  WITH_ENERGY_EFFICIENCY

             process_post(PROCESS_BROADCAST, energy_efficient_begin_event, NULL);

           #endif
         }
      }   
// if node is root node keep working in active state forever.
  
#if (!ROOTNODE) 
    if(active_flag ==1 || low_latency_flag == 1){    
   //  P3OUT &= ~0x40; // led on
      TA1CCTL1 |= CCIE;

      //zhangwei set changed for load balance
     if(low_latency_flag == 0){
        #if  WITH_ENERGY_EFFICIENCY
          rdc_efficiency_request_poll();
        #endif
        ++rtc_second;
      } 
     // TA1CCTL0 |= CCIE;
    }else {         
   //  P3OUT |=0x40;            //led   off
     TA1CCTL1 &= ~(CCIE);     //etimer  off
     
     // TA1CCTL0 &= ~(CCIE);     // rtimer off

    /*   
      NETSTACK_RADIO.get_value(RADIO_PARAM_POWER_MODE, &radio_status);
      if(radio_status==RADIO_POWER_MODE_OFF && timenow.sec==3){
           rttt=RTIMER_NOW();
      if(cpu_status())
           TA1CCTL0 &= ~(CCIE);
      }
      LPM3; *//* LPM3 sleep. This*/

   }
#endif
    break;
    case RTC_RTCAIFG:                       // RTCAIFG
      break;

    case RTC_RT0PSIFG:                      // RT0PSIFG
      break;

    case RTC_RT1PSIFG:                      // RT1PSIFG
      break;

    case 12: break;                         // Reserved
    case 14: break;                         // Reserved
    case 16: break;                         // Reserved
    default: break;
  }
  
  watchdog_stop();

  ENERGEST_ON(ENERGEST_TYPE_LPM);

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void set_init_flag(uint16_t flag){
    init_net_flag =flag;
}

void set_ledon_flag(uint8_t flag){
  ledon_flag = flag;
}

uint16_t get_init_flag(){
  return init_net_flag ;
}

void set_active_flag(int hour,int minute,int second_s)
{
  //calendar_time  cal_time_now;
  int index=0;
  // read_calendar(&cal_time_now);
  // int hour   = BCD_to_dec(cal_time_now.hour);
  // int minute = BCD_to_dec(cal_time_now.min);
  index=hour*6+minute/10;         //6   ,10
  schedule_bitmap_get(schedule_bit);
#if 0  
  int index2=0;
  for(;index2<18;index2++){
    printf("schedule_bit [%d] = %d\n",index2,schedule_bit[index2] );
  }
#endif
  
#if UIP_CONF_AUTO_SLEEP
  if(get_round_auto_sleep() != index){
    clear_force_auto_sleep();
    set_round_auto_sleep(index);
  }
#endif

#if UIP_CONF_AUTO_SLEEP
  if(get_force_auto_sleep()){
     active_flag = 0;
  }else{
#endif  
    active_flag= init_net_flag&((schedule_bit[index/8]) >> (7-(index%8)));

#if UIP_CONF_AUTO_SLEEP 
  }
#endif

  #if !ROOTNODE
//  printf("active flag:%u\n",active_flag);
  #endif
//  active_flag = 1;
}

void syn_update_timenow(soft_time syntime)
{
   timenow.minute=syntime.minute;
   timenow.hour  =syntime.hour;
   timenow.sec   =syntime.sec;
}

void get_timenow(soft_time *times)
{
   times->hour   =timenow.hour;
   times->minute =timenow.minute;
   times->sec    =timenow.sec;
}

clock_time_t get_idle_time(void)  //return 0 means not in active mode .
{
  uint8_t counter=0;
  int index=0;
  int i=0;

  // get_timenow(&timenow);
  if(active_flag){
    index=timenow.hour*6 +timenow.minute/10 +1;
      // printf("index :%d\n",index);
    for(;i<144;i++){
      if((((schedule_bit[index/8]) >> (7-(index%8)) ) & 0x01)){
             index=(index+1)%144;
             ++counter;
    }else 
      break;
    }
    // printf("%d %d\n",counter,index);
    return ((10*counter+9-timenow.minute%10)*60+(60-timenow.sec)); 
  }
  return 0x0;
}

uint8_t get_active_flag(void)
{
  return active_flag;
}

/*----------------------------------------------------------------*/
uint16_t 
get_nowdays()
{
  return days;
}
/*----------------------------------------------------------------*/

void 
set_autocal_info(int autocal_offest,uint32_t autocal_interval){
  cal_count = 0;
  cal_offest = autocal_offest;
  cal_interval = autocal_interval;
}

#endif

#if 0
int BCD_to_dec(uint8_t x)
{
   x=(x & 0x0f)+((x>>4) &0x07)*10;
   return x;
}

static void write_rtcamin(void)
{

   for(;(RTCCTL01&RTCRDY););
   for(;!(RTCCTL01&RTCRDY););

  RTCCTL01 |=  RTCHOLD ;
   
    RTCAMIN = (((RTCMIN & 0x70)+0x10)% 0x60)|0x80;
  
  RTCCTL01 &= ~(RTCHOLD);    

}
#endif
#if 0
void read_calendar(calendar_time * caltime) //notice: calendar mode , register BCD mode. 
{
    for(;(RTCCTL01&RTCRDY););
    for(;!(RTCCTL01&RTCRDY););

   caltime->year= RTCYEAR          ;// Year 
   caltime->mon= RTCMON            ;// Month 
   caltime->day= RTCDAY            ;// Day 
   caltime->day_of_week= RTCDOW    ;// Day of week 
   caltime->hour= RTCHOUR          ;// Hour 
   caltime->min= RTCMIN            ;// Minute 
   caltime->sec= RTCSEC            ;// Seconds 
     
}
void write_calendar(calendar_time caltime)
{
   for(;(RTCCTL01&RTCRDY););
   for(;!(RTCCTL01&RTCRDY););

  // dint ();
  RTCCTL01 |= RTCHOLD;

  RTCYEAR = caltime.year  ;        // Year  
  RTCMON  =  caltime.mon  ;        // Month   
  RTCDAY  =  caltime.day  ;        // Day   
  RTCDOW  =  caltime.day_of_week ; // Day of week   
  RTCHOUR = caltime.hour  ;        // Hour 
  RTCMIN  =  caltime.min  ;        // Minute 
  RTCSEC  =  caltime.sec  ;        // Seconds 
  
  P3OUT &=~0x40;// light on the led;
  
  RTCAMIN = (((RTCMIN & 0xcounter70)+0x10)% 0x60)|0x80;

  RTCCTL01 &= ~(RTCHOLD);    
  // eint ();

}
#endif /*TRXEB1120_CONF_LOWPOWER */ 

//zhangwei set changed for load balance
unsigned long 
real_time_clock_second(void)
{
  unsigned long t1, t2;
  do {
    t1 = rtc_second;
    t2 = rtc_second;
  } while(t1 != t2);
  return t1;
}
#if 0
int
cpu_status(void)
{
  /* Clear the GIE (General Interrupt Enable) flag. */
  int sr;
#ifdef __IAR_SYSTEMS_ICC__
  sr = __get_SR_register();
  __bic_SR_register(GIE);
#else
  asmv("mov r2, %0" : "=r" (sr));
  asmv("bic %0, r2" : : "i" (CPUOFF));
#endif
  return sr & CPUOFF;              /* Ignore other sr bits. */
}
#endif

  //update rtcamin
      //RTCAMIN=(((((RTCAMIN & 0x0f)+((RTCAMIN >>4) & 0x07)*10+RTC_CAL_MIN_INTERVAL) % 60)/10)*6)+(((RTCAMIN & 0x0f)+((RTCAMIN >>4) & 0x07)*10+RTC_CAL_MIN_INTERVAL) % 60);
      //RTCAMIN |= 0x80;                           // RTC Minute Alarm
 // RTCAMIN= (((RTCMIN & 0x70)+0x10)% 0x60)|0x80;


//write_rtcamin();                      // write_rtcamin 
// printf("alarm: %x,%x,%x ;\n",RTCHOUR,RTCMIN,RTCSEC); 
