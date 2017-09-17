/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: auto-sleep.c ,v 1.21 2017/03/25 22:46:15 
*/
/**
 * \file
 *        node auto sleep.
 * \author
 *        @yang
 */

#include "auto-sleep.h"
#include "contiki.h"

#include "netsynch.h"

#if UIP_CONF_IPV6_RPL
#include "rpl/rpl.h"
#endif

#define DEBUG  0  //DEBUG_PRINT
#include "net/ip/uip-debug.h"

#if UIP_CONF_AUTO_SLEEP

#define AUTO_SLEEP_DELAY  (120*CLOCK_SECOND)

static uint8_t force_auto_sleep = 0;
static int round_auto_sleep =-1;
static struct ctimer auto_sleep_ct;
/*---------------------------------------------------------------------------*/
static void
set_force_auto_sleep(void *p) {
#if 1
  if(rpl_get_any_dag() == NULL && netsynch_is_synched()) {
    printf("auto enter sleep\n");
    force_auto_sleep = 1;
  }
#endif
}
/*---------------------------------------------------------------------------*/
void
clear_force_auto_sleep(void){
  force_auto_sleep = 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
get_force_auto_sleep(void){
  return force_auto_sleep;
}
/*---------------------------------------------------------------------------*/
void
set_round_auto_sleep(int round_time){
  round_auto_sleep = round_time;
}
/*---------------------------------------------------------------------------*/
int
get_round_auto_sleep(void){
  return round_auto_sleep;
}
/*---------------------------------------------------------------------------*/
void
set_delay_auto_sleep(clock_time_t sleep_delay){
  ctimer_set(&auto_sleep_ct,sleep_delay,set_force_auto_sleep,NULL);
}
/*---------------------------------------------------------------------------*/
void
packet_input_auto_sleep(void){
  if(rpl_get_any_dag() != NULL){
    //clear_auto_sleep();
    /* delay time is not suitbal */

   // ctimer_set(&auto_sleep_ct,AUTO_SLEEP_DELAY,set_force_auto_sleep,NULL);
  }
}
/*---------------------------------------------------------------------------*/
#endif

