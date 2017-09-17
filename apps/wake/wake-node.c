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

#define DEBUG  1  //DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define MCAST_WAKE_UDP_PORT 5701   /* Host byte order */
static struct uip_udp_conn * mcast_conn;
/*---------------------------------------------------------------------------*/
PROCESS(wake_node_process, "wake node process");
/*---------------------------------------------------------------------------*/
static void
prepare_mcast(void){
  mcast_conn = udp_new(NULL, UIP_HTONS(MCAST_WAKE_UDP_PORT), NULL);
  udp_bind(mcast_conn, UIP_HTONS(MCAST_WAKE_UDP_PORT));
}
/*---------------------------------------------------------------------------*/
void
wake_send(void *ptr)
{
  uip_ipaddr_t ipaddr;
  struct wake_st *wake_tmp = get_wake_info();

  if(wake_tmp->wake_len >0){
    printf("wake---------\n");
    set_wake_send(1);
    uip_create_linklocal_allnodes_mcast(&ipaddr);
    uip_udp_packet_sendto(mcast_conn, wake_tmp->wake_buf, wake_tmp->wake_len,
                       &ipaddr, UIP_HTONS(MCAST_WAKE_UDP_PORT));
  }
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {
    uint8_t non_frag_flag = 0;
    uint8_t wake_channel = 0;
    uint16_t wake_type;
    uint8_t wake_seq;
    int pos = 0;
    struct wake_st *wake_info =NULL;

#if SUBNET_PANID_CONF_LIMIT & 0
    /* drop not our panid packet. not right */
      if(!subnet_panid_limit(get_link_receive_subnet_panid())) {
         return;
      }
#endif

    non_frag_flag = uip_udp_received_data_preprocess();
    if(!non_frag_flag)
      return;
    
    printf("receive wake info\n");
    memcpy(&wake_type, ((uint16_t *)(uip_appdata)), sizeof(wake_type));
    //wake_type = UIP_HTONS(wake_type);
    pos +=  sizeof(wake_type);
    printf("type:%04x\n",wake_type);
    if(wake_type == WAKE_NODE_TYPE){
      memcpy(&wake_seq, ((uint8_t *)(uip_appdata+pos)), sizeof(wake_seq));  
      pos += sizeof(wake_seq);
      memcpy(&wake_channel, ((uint8_t *)(uip_appdata+pos)), sizeof(wake_channel));  
      pos += sizeof(wake_channel);

      wake_info = get_wake_info();
      if(wake_info->wake_seq == wake_seq){
        return;
      }

      leds_on(LEDS_GREEN);
      printf("wake_channel : %u\n",wake_channel);
      normalbyte_rfchannel_burn(1,wake_channel);
      set_init_flag(1);

      cache_wake_info(wake_seq,uip_appdata,pos);
      wake_send(NULL);  

      //NodeReboot(NULL);
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


