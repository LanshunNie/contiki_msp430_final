/**
 * \addtogroup wake dev
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: wake-dev.c,v 1.4 2016/10/06 19:36:15  
 */

/**
 * \file
 *         wake dev
 * \author
 *         @yang
 */

#include "wake-common.h"
#include "wake-node.h"
#include "contiki.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/rpl/rpl.h"

#include "dev/leds.h"

#include <stdio.h>
#include <string.h>

#define DEBUG  0  //DEBUG_PRINT
#include "net/ip/uip-debug.h"

//#define START_INTERVAL  (CLOCK_SECOND *2)
//static struct ctimer wake_dev_send_ct;
static uint8_t wake_seq;
static uint16_t wake_type;
/*---------------------------------------------------------------------------*/
void 
wake_dev_send(void *p){
  uint8_t buff[20];
  int pos =0;

  memset(buff , 0, sizeof(buff));
  wake_seq++;
  if(wake_seq == 0){
    wake_seq++;
  }

  memcpy(buff+pos, &wake_type, sizeof(wake_type));
  pos += sizeof(wake_type);
  memcpy(buff+pos, &wake_seq, sizeof(wake_seq));
  pos += sizeof(wake_seq);
  buff[pos] = WAKEUP_NODE_RFCHANNEL ;
  pos++;

  //leds_toggle(LEDS_GREEN);

  //ctimer_stop(&wake_dev_send_ct);
  cache_wake_info(wake_seq,buff,pos);
  wake_send(NULL);
}
/*---------------------------------------------------------------------------*/
void
wake_dev_init(void){
  //ctimer_set(&wake_dev_send_ct,START_INTERVAL,wake_dev_send,NULL);
  wake_seq = 0;
  wake_type = WAKE_NODE_TYPE;
}
/*---------------------------------------------------------------------------*/

