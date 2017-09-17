/**
 * \addtogroup wake common
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: wake-dev.c,v 1.4 2017/07/21 17:10:15  
 */

/**
 * \file
 *         wake common
 * \author
 *         @yang
 */

#include "wake-common.h"
#include "contiki.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "dev/leds.h"

#include "wake-dev.h"
#include "wake-node.h"
 
#include <stdio.h>
#include <string.h>

#define DEBUG  0  //DEBUG_PRINT
#include "net/ip/uip-debug.h"

/* Constants of the IEEE 802.15.4 standard */

/* MinBE: Initial backoff exponent. Range 0--WAKE_MAX_BE */
#ifdef WAKE_CONF_MIN_BE
#define WAKE_MIN_BE WAKE_CONF_MIN_BE
#else
#define WAKE_MIN_BE 0
#endif

/* MaxBE: Maximum backoff exponent. Range 3--8 */
#ifdef WAKE_CONF_MAX_BE
#define WAKE_MAX_BE WAKE_CONF_MAX_BE
#else
#define WAKE_MAX_BE 4
#endif

/* MaxWAKEBackoffs: Maximum number of backoffs in case of channel busy/collision. Range 0--5 */
#ifdef WAKE_CONF_MAX_BACKOFF
#define WAKE_MAX_BACKOFF WAKE_CONF_MAX_BACKOFF
#else
#define WAKE_MAX_BACKOFF 10
#endif

static struct ctimer transmit_ct;
static uint8_t wake_send_flag;
static struct wake_st wake_info;
/*---------------------------------------------------------------------------*/
static clock_time_t
backoff_period(void)
{
  return (NETSTACK_RDC_CHANNEL_CHECK_INTERVAL+1) * CLOCK_SECOND;
}
/*---------------------------------------------------------------------------*/
static void
schedule_transmission(struct wake_st *n)
{
  clock_time_t delay;
  int backoff_exponent; /* BE in IEEE 802.15.4 */

  backoff_exponent = MIN(n->wake_collisions, WAKE_MAX_BE);

  /* Compute max delay as per IEEE 802.15.4: 2^BE-1 backoff periods  */
  delay = ((1 << backoff_exponent) - 1) * backoff_period();
  if(delay > 0) {
    /* Pick a time for next transmission */
    delay = backoff_period()/2 + random_rand() % delay;
  }

  PRINTF("WAKE: scheduling transmission in %u ticks, NB=%u, BE=%u\n",
      (unsigned)delay, n->wake_collisions, backoff_exponent);

  ctimer_set(&transmit_ct, delay, wake_send, NULL);
}
/*---------------------------------------------------------------------------*/
static void
free_packet(struct wake_st *n)
{
  set_wake_send(0);
  n->wake_len =0;
  n->wake_collisions =0;
}
/*---------------------------------------------------------------------------*/
static void
tx_done(int status)
{
  free_packet(&wake_info);
}
/*---------------------------------------------------------------------------*/
static void
rexmit(struct wake_st *n)
{
  schedule_transmission(n);
}
/*---------------------------------------------------------------------------*/
static void
collision(int num_transmissions,struct wake_st *n)
{
  n->wake_collisions += num_transmissions;

  if(n->wake_collisions > WAKE_MAX_BACKOFF) {
    tx_done(MAC_TX_COLLISION);
  } else {
    PRINTF("WAKE: rexmit collision %d\n", n->wake_collisions);
    rexmit(n);
  }
}
/*---------------------------------------------------------------------------*/
static void
tx_ok(int num_transmissions,struct wake_st *n)
{
  free_packet(n);
}
/*---------------------------------------------------------------------------*/
void 
cache_wake_info(uint8_t seq,uint8_t buf[],uint8_t len){
  wake_info.wake_len =len;
  wake_info.wake_collisions =0;
  wake_info.wake_seq = seq;
  memcpy(wake_info.wake_buf,buf,len);
}
/*---------------------------------------------------------------------------*/
struct wake_st*
get_wake_info(void){
  return &wake_info;
}
/*---------------------------------------------------------------------------*/
void
wake_sent_callback(int status, int num_transmissions){
  set_wake_send(0);
  switch(status) {
  case MAC_TX_OK:
    tx_ok(num_transmissions,&wake_info);
    break;
  case MAC_TX_NOACK:
    tx_done(status);
    break;
  case MAC_TX_COLLISION:
    collision(num_transmissions,&wake_info);
    break;
  case MAC_TX_DEFERRED:
    break;
  default:
    tx_done(status);
    break;
  }
}
/*---------------------------------------------------------------------------*/
void 
set_wake_send(uint8_t flag){
  wake_send_flag =flag;
}
/*---------------------------------------------------------------------------*/
uint8_t
get_wake_send(void)
{
  return wake_send_flag;
}
/*---------------------------------------------------------------------------*/
void
wake_common_init(void){
  set_wake_send(0);
  wake_info.wake_len =0;
  wake_info.wake_collisions =0;
  wake_info.wake_seq = 0;
  wake_dev_init();
  wake_node_init();
}
/*---------------------------------------------------------------------------*/


