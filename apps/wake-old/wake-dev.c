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

#if WAKEUP_NODE_DEV
#define MCAST_WAKE_UDP_PORT 5701   /* Host byte order */
#define START_INTERVAL  (CLOCK_SECOND *2)

static struct uip_udp_conn * mcast_conn;
static struct ctimer wake_dev_send_ct;
/*---------------------------------------------------------------------------*/
PROCESS(wake_dev_process, "wake dev process");
/*---------------------------------------------------------------------------*/
#if WAKEUP_NODE_DEV_FLAG
static void 
wake_dev_send(void *p){
  static uint8_t wake_array[] = {0x39,0x5A,0x6B,0x7E,0x4C};

  uip_ipaddr_t ipaddr;
  uint8_t buff[20];

  memset(buff , 0, sizeof(buff));
  
  memcpy(buff, wake_array, sizeof(wake_array));
  buff[ sizeof(wake_array)] = WAKEUP_NODE_RFCHANNEL ;
  
  leds_toggle(LEDS_GREEN);

  ctimer_stop(&wake_dev_send_ct);
  uip_create_linklocal_allnodes_mcast(&ipaddr);
  uip_udp_packet_sendto(mcast_conn, buff, sizeof(wake_array) + 1,
                       &ipaddr, UIP_HTONS(MCAST_WAKE_UDP_PORT));
}
#endif
/*---------------------------------------------------------------------------*/
#if UNWAKEUP_NODE_DEV_FLAG
static void 
wake_dev_send(void *p){
  static uint8_t unwake_array[] = {0x89,0x7A,0x5C,0x49,0xBD};

  uip_ipaddr_t ipaddr;
  uint8_t buff[20];

  memset(buff , 0, sizeof(buff));
  
  memcpy(buff, unwake_array, sizeof(unwake_array));
  buff[ sizeof(unwake_array)] = 0 ;
  
  leds_toggle(LEDS_GREEN);

  ctimer_stop(&wake_dev_send_ct);
  uip_create_linklocal_allnodes_mcast(&ipaddr);
  uip_udp_packet_sendto(mcast_conn, buff, sizeof(unwake_array) + 1,
                       &ipaddr, UIP_HTONS(MCAST_WAKE_UDP_PORT));
}
#endif
/*---------------------------------------------------------------------------*/
#if CHECK_WAKEUP_DEV_FLAG
static void 
wake_dev_send(void *p){
  static uint8_t checkwake_array[] = {0x7C,0x43,0x83,0x1F,0x6D};

  uip_ipaddr_t ipaddr;
  uint8_t buff[20];

  memset(buff , 0, sizeof(buff));
  
  memcpy(buff, checkwake_array, sizeof(checkwake_array));
  buff[ sizeof(checkwake_array)] = 0 ;
  
  leds_toggle(LEDS_GREEN);

  ctimer_stop(&wake_dev_send_ct);
  uip_create_linklocal_allnodes_mcast(&ipaddr);
  uip_udp_packet_sendto(mcast_conn, buff, sizeof(checkwake_array) + 1,
                       &ipaddr, UIP_HTONS(MCAST_WAKE_UDP_PORT));
}
#endif
/*---------------------------------------------------------------------------*/
static void
prepare_mcast(void){
  mcast_conn = udp_new(NULL, UIP_HTONS(MCAST_WAKE_UDP_PORT), NULL);
  udp_bind(mcast_conn, UIP_HTONS(MCAST_WAKE_UDP_PORT));
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
}
/*---------------------------------------------------------------------------*/
void
wake_dev_init(void){
  ctimer_set(&wake_dev_send_ct,START_INTERVAL,wake_dev_send,NULL);
  process_start(&wake_dev_process,NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wake_dev_process, ev, data)
{
  PROCESS_BEGIN();
 
  prepare_mcast();
  // ctimer_set(&wake_dev_send_ct,START_INTERVAL,wake_dev_send,NULL);

  while(1) {
    PROCESS_YIELD();

    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#endif /* WAKEUP_NODE_DEV */
