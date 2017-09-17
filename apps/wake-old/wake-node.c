/**
 * \addtogroup wake node
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: wake-node.c,v 1.4 2016/10/06 19:36:15  
 */

/**
 * \file
 *         wake node
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

#include "node_function.h"
#include "cc1120.h"
#include "clock.h"
#include "node-id.h"
#include "dev/leds.h"


#include <stdio.h>
#include <string.h>

#define DEBUG  ROOTNODE  //DEBUG_PRINT
#include "net/ip/uip-debug.h"

#if !WAKEUP_NODE_DEV
#define MCAST_WAKE_UDP_PORT 5701   /* Host byte order */

static struct uip_udp_conn * mcast_conn;
static uint8_t wake_array[] = {0x39,0x5A,0x6B,0x7E,0x4C};
static uint8_t unwake_array[] = {0x89,0x7A,0x5C,0x49,0xBD};
static uint8_t checkwake_array[] = {0x7C,0x43,0x83,0x1F,0x6D};
static struct ctimer ledoff_ct;
/*---------------------------------------------------------------------------*/
PROCESS(wake_node_process, "wake node process");
/*---------------------------------------------------------------------------*/
static void 
led_off_callback(void *p){
  leds_off(LEDS_ALL);
  set_ledon_flag(0);
}
/*---------------------------------------------------------------------------*/
static void
prepare_mcast(void){
  mcast_conn = udp_new( NULL, UIP_HTONS(MCAST_WAKE_UDP_PORT), NULL);
  udp_bind(mcast_conn, UIP_HTONS(MCAST_WAKE_UDP_PORT));
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {
    uint8_t non_frag_flag = 0;
    uint8_t wake_channel = 0;
    
#if SUBNET_PANID_CONF_LIMIT & 0
    /* drop not our panid packet. not right */
      if(!subnet_panid_limit(get_link_receive_subnet_panid())) {
         return;
      }
#endif

    non_frag_flag = uip_udp_received_data_preprocess();
    if(!non_frag_flag)
      return;
    
    PRINTF("receive wake info\n");
    if(memcmp(uip_appdata, wake_array, sizeof(wake_array)) == 0){
      wake_channel = ((uint8_t *)uip_appdata)[ sizeof(wake_array)];
      PRINTF("wake_channel : %u\n",wake_channel);
      normalbyte_rfchannel_burn(1,wake_channel);
      set_init_flag(1);
      NodeReboot(NULL);
    //  cc1120_channel_set(wake_channel);
    }else if(memcmp(uip_appdata, unwake_array, sizeof(unwake_array)) == 0){
      PRINTF("receive unwake message\n");
      NodeReset(NULL);
     //  normalbyte_rfchannel_burn(0,0);
     // // set_init_flag(0);
     //  NodeReboot(NULL);
    }else if(memcmp(uip_appdata, checkwake_array, sizeof(checkwake_array)) == 0){
      PRINTF("receive check wake message\n");
      if(get_init_flag()){
        set_ledon_flag(1);
        leds_on(LEDS_GREEN);
        ctimer_set(&ledoff_ct,CLOCK_SECOND,led_off_callback ,NULL);
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
void
wake_node_init(void){
  process_start(&wake_node_process,NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wake_node_process, ev, data)
{
  PROCESS_BEGIN();
  
  prepare_mcast();
  
  while(1) {
    PROCESS_YIELD();

    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#endif /* !WAKEUP_NODE_DEV */

