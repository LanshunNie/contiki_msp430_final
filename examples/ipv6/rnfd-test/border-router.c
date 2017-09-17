/*
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
 *
 */
/**
 * \file
 *         border-router
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Nicolas Tsiftes <nvt@sics.se>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/rpl/rpl.h"
#include "dev/leds.h"
#include "net/netstack.h"
#include "dev/slip.h"
 
#include "simple-udp.h"
#include "node_function.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "net/ipv6/multicast/uip-mcast6.h"
 
#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define MCAST_SINK_UDP_PORT 3001

/* Start sending messages START_DELAY secs after we start so that routing can
 * converge */
#define START_DELAY  60

#define BASE_TIME  (120 * CLOCK_SECOND)      //120
#define RANDOM_TIME  (4 * CLOCK_SECOND)      // 4

//static uint16_t seq_id = 0;
#define MAX_PAYLOAD_LEN 128


static struct uip_udp_conn *server_conn;
static struct uip_udp_conn * mcast_conn;
static struct ctimer ct;
static struct ctimer ct1;

static int multicast_send_flag = 1;

static char buffer[MAX_PAYLOAD_LEN];
static unsigned char buffered_data_length = 0;

static uip_ipaddr_t prefix;
static uint8_t prefix_set;

static void  msg_handler(char *appdata,int appdata_length);
static void root_handle_message(void*p);
/*--------------------------------------------------------*/
PROCESS(border_router_process, "Border router process");
AUTOSTART_PROCESSES(&border_router_process);
/*---------------------------------------------------------------------------*/
#if 0
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTA("Server IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      //PRINTA(" ");
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
      //PRINTA("\n");
    }
  }
}
#endif
/*---------------------------------------------------------------------------*/
#if 0
static void
multicast_send(void)
{
  uint8_t buf[10];
  memset(buf, 0, sizeof(buf));
  int pos = 0;
  soft_time times; 

  get_timenow(&times);
  seq_id++;

  memcpy(buf, &seq_id, sizeof(seq_id));
  pos += sizeof(seq_id);

  buf[pos++] = times.hour;
  buf[pos++] = times.minute;
  buf[pos++] = times.sec;

  printf("multicast send seq_id:%u\n",seq_id);

  uip_udp_packet_send(mcast_conn, buf, pos);
}
#endif
/*---------------------------------------------------------------------------*/
static void
prepare_mcast(void)
{
  uip_ipaddr_t ipaddr;

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
   //建立多播组发送连接
  uip_ip6addr(&ipaddr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  mcast_conn = udp_new(&ipaddr, UIP_HTONS(MCAST_SINK_UDP_PORT), NULL);
}
/*---------------------------------------------------------------------------*/
void
request_prefix(void)
{
  /* mess up uip_buf with a dirty request... */
  uip_buf[0] = '?';
  uip_buf[1] = 'P';
  uip_len = 2;
  slip_send();
  uip_len = 0;
}
/*---------------------------------------------------------------------------*/
void
set_prefix_64(uip_ipaddr_t *prefix_64)
{
  rpl_dag_t *dag;
  uip_ipaddr_t ipaddr;
  memcpy(&prefix, prefix_64, 16);
  memcpy(&ipaddr, prefix_64, 16);
  prefix_set = 1;
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
  dag = rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
  if(dag != NULL) {
    rpl_set_prefix(dag, &prefix, 64);
    printf("a new RPL dag\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
multicast_send(void * p)
{
  if(buffered_data_length){
    printf("root node multicast_send\n");
    multicast_send_flag = 1;
    uip_udp_packet_send(mcast_conn, buffer, buffered_data_length + 1);

    leds_toggle(LEDS_ALL);
  
    #if LOW_LATENCY
       ctimer_set(&ct,CLOCK_SECOND*2,root_handle_message,NULL);
    #endif
  }
}
/*---------------------------------------------------------------------------*/
static void 
root_handle_message(void*p){
  msg_handler(buffer+2,buffered_data_length-1);
}
/*---------------------------------------------------------------------------*/
static void  
msg_handler(char *appdata,int appdata_length){
  uint8_t type=(uint8_t)appdata[0];
  switch(type)
  {
    case CMD_REBOOT:
      ctimer_set(&ct1, (30 * CLOCK_SECOND),NodeReboot,NULL); 
    break;

    default:                 
      break;
  }
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{

  if(uip_newdata()) {
    buffered_data_length = (unsigned char) ((char *)uip_appdata)[0];
    leds_toggle(LEDS_ALL);

    //sprintf(buf, "Hello %d", seq_id);//封装在buffer里
    if((UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2])==0
        &&(UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1])==2){

      memset(buffer, 0, MAX_PAYLOAD_LEN);
      memcpy(buffer, (char *)uip_appdata, buffered_data_length + 1);

      printf("send type:%x-%x\n",(uint8_t)(*(buffer+1)),(uint8_t)(*(buffer+2)) );

      if(get_idle_time() >=2* 60){      
        multicast_send(NULL);
      }

      #if LOW_LATENCY
        
        if(get_lowLatency_flag() ==1){
          if(get_low_latency_active_time() < 8){
            low_latency_msg_send_register(multicast_send);
          }else{
            multicast_send(NULL);
          }
        }else{
          if(get_lowLatency_flag() == 0 && get_active_flag() == 0){
            low_latency_msg_send_register(multicast_send);
          }
        }
      #else
        ctimer_set(&ct,CLOCK_SECOND*10,root_handle_message,NULL);
      #endif   
    }
   
#if SERVER_REPLY
    //printf("DATA sending reply\n");
    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    uip_udp_packet_send(server_conn, "Reply", sizeof("Reply"));
    uip_create_unspecified(&server_conn->ripaddr);
#endif
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(border_router_process, ev, data)
{
  static struct etimer et;
  PROCESS_BEGIN();
  
/* While waiting for the prefix to be sent through the SLIP connection, the future
 * border router can join an existing DAG as a parent or child, or acquire a default
 * router that will later take precedence over the SLIP fallback interface.
 * Prevent that by turning the radio off until we are initialized as a DAG root.
 */
  prefix_set = 0;

  NETSTACK_MAC.off(0);//csma

  PROCESS_PAUSE();

  /* Request prefix until it has been received */
  while(!prefix_set) 
  {
    etimer_set(&et, CLOCK_SECOND);
    request_prefix();
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  prepare_mcast();
  /* Now turn the radio on, but disable radio duty cycling.
   * Since we are the DAG root, reception delays would constrain mesh throughbut.
   */
  NETSTACK_MAC.off(1);
  // print_local_addresses();

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(server_conn == NULL) {
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      leds_toggle(LEDS_GREEN);
      tcpip_handler();
    }   
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


