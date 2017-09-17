/**
 * \addtogroup orpl Low latency msg control for root transmited mechanism process
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: low-latency-msg.c,v 1.21 2017/04/10 10:20:00
 */

/**
 * \file
 *        A simple Low latency msg control for root mechanism
 * \author
 *         zhangwei
 */


#include "contiki.h"
#include "low-latency-msg.h"
#include "low-latency.h"
#include <stdio.h>

/*---------------------------------------------------------------------------*/
#define SEND_LATENCY 3

static void (*low_latency_send)(void *) = NULL;
static struct ctimer ll_send_ct;
static struct ctimer out_send_ct;
static void low_latency_msg_send_logout(void *p);
/*---------------------------------------------------------------------------*/
static void low_latency_msg_send_logout(void *p){
  if(low_latency_send != NULL)
    low_latency_send =NULL;
}
/*---------------------------------------------------------------------------*/
void 
low_latency_msg_send(void){
  if(low_latency_send){

    ctimer_set(&ll_send_ct , SEND_LATENCY *CLOCK_SECOND,low_latency_send,NULL);
    ctimer_set(&out_send_ct , (SEND_LATENCY+1) *CLOCK_SECOND,low_latency_msg_send_logout,NULL);

  }
}

/*---------------------------------------------------------------------------*/

void 
low_latency_msg_send_register(void (*f)(void *)){
  
  if(f!=NULL){
    low_latency_send = f;
  }
}

/*---------------------------------------------------------------------------*/