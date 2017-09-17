/**
 * \addtogroup orpl Low latency mechanism process
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: low-latency.c,v 1.21 2017/03/31 20:00:00
 */

/**
 * \file
 *        A simple Low latency mechanism
 * \author
 *         zhangwei
 */

#include "contiki.h"
#include "low-latency.h"
#include "net/mac/contikimac/contikimac.h"
#include <stdio.h>

#define DEBUG 0

#if ROOTNODE | DEBUG
#include "low-latency-msg.h"
#endif

/*---------------------------------------------------------------------------*/
// #define ACTIVE_CURRENT 750000//nA
// #define INACTIVE_CURRENT 4250 //nA
// #define LOW_LATENCY_ACTIVE_CURRENT 300000//nA
// #define CURRENT_BUDGET   25000//nA
// #define ACTIVE_COUNT   2

/*---------------------------------------------------------------------------*/
static uint8_t previous_index = 0xFF;
static uint8_t index_change_flag = 0;
static uint8_t low_latency_falg =0;
static uint8_t low_latency_channel_clear_flag = 1;
static uint8_t low_latency_interval = 10;
static uint8_t low_latency_active_second = 20;
static struct ctimer auto_sleep_ctimer;
static int8_t low_latency_active_time = 0;
/*---------------------------------------------------------------------------*/
// static void auto_sleep_callback(void *p);
// static void cal_base_conf(void);
/*---------------------------------------------------------------------------*/
/*static void 
cal_base_conf(void){
  uint8_t ll_lount = (144 * CURRENT_BUDGET - ACTIVE_CURRENT * ACTIVE_COUNT - INACTIVE_CURRENT * (144 - ACTIVE_COUNT))                              \
                      / (LOW_LATENCY_ACTIVE_CURRENT - INACTIVE_CURRENT);

  uint8_t per_ten_min_sec = (ll_lount * 10 * 60) /(144 - ACTIVE_COUNT);

  low_latency_active_second = per_ten_min_sec * low_latency_interval / 10;
}
*/
/*---------------------------------------------------------------------------*/
static void 
auto_sleep_callback(void *p){

  if(low_latency_channel_clear_flag == 0){

    low_latency_falg = 0;
    low_latency_active_time =0;
    index_change_flag = 0;
  }else{

    low_latency_falg = 1;

  }

  // printf("hello\n");
}

/*---------------------------------------------------------------------------*/
uint8_t 
set_lowLatency_flag(soft_time  timenow){

  if(get_active_flag() == 0){

    uint8_t index = timenow.hour*6+timenow.minute/low_latency_interval;
  
    if(index != previous_index){

      low_latency_active_time = low_latency_active_second;

      previous_index = index;
      index_change_flag = 1;
      low_latency_channel_clear_flag = 0;

      set_cycletime(RTIMER_ARCH_SECOND/get_rdc_active_channel_check_rate());

      #if ROOTNODE | DEBUG
        low_latency_msg_send();
      #endif

      ctimer_set(&auto_sleep_ctimer , low_latency_active_second * 4 / 5 * CLOCK_SECOND,auto_sleep_callback,NULL);
    }

    if(index_change_flag == 1 && timenow.sec / low_latency_active_second == 0){

      low_latency_active_time--;

      low_latency_falg = 1;

      // printf("low_latency_active_time %u \n",low_latency_active_time );
     // ctimer_set(&auto_sleep_ctimer , CLOCK_SECOND / 2,auto_sleep_callback,NULL);

    }else{
      low_latency_active_time = 0;
      low_latency_channel_clear_flag = 0;
      index_change_flag =0;
      low_latency_falg = 0;
    }
  }else {
    low_latency_active_time = 0;
    low_latency_channel_clear_flag = 0; 
    index_change_flag =0;
    low_latency_falg = 0;

  }

  return low_latency_falg;
  
}
/*---------------------------------------------------------------------------*/
uint8_t 
get_lowLatency_flag(void){
  return low_latency_falg;
}
/*---------------------------------------------------------------------------*/
void 
set_low_latency_interval(uint8_t _interval){

  low_latency_interval = _interval;

}

/*---------------------------------------------------------------------------*/
uint8_t 
get_low_latency_interval(void){

  return low_latency_interval;

}

/*---------------------------------------------------------------------------*/
void 
set_low_latency_active_second(uint8_t _second){

  low_latency_active_second = _second;

}

/*---------------------------------------------------------------------------*/
uint8_t 
get_low_latency_active_second(void){

  return low_latency_active_second;

}
/*---------------------------------------------------------------------------*/

void 
set_low_latency_channel_clear_flag(uint8_t _packet_seen){

  low_latency_channel_clear_flag = _packet_seen;

}
/*---------------------------------------------------------------------------*/

int8_t 
get_low_latency_active_time(void){

  return low_latency_active_time;

}
/*---------------------------------------------------------------------------*/