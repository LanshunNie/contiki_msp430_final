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

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"
#include "net/linkaddr.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#if HW_CONF_WITH_UART1
#include "dev/uart1.h"
#else
#include "dev/uart0.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "collect-common.h"
#include "collect-view.h"

#if UIP_CONF_IPV6_ORPL_BITMAP
//#include "net/rpl-bitmap/orpl-bitmap.h"
#include "net/orpl-bitmap/orpl-routing-set.h"
#endif

#include "simple-udp.h"
#include "dev/leds.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

static struct uip_udp_conn *server_conn;


#define MAX_PAYLOAD_LEN 100
#define UCAST_SINK_UDP_PORT 3001 /* Host byte order */
#define UDP_PORT  3002
#define SEND_INTERVAL CLOCK_SECOND /* clock ticks */
//#define ITERATIONS   200 /* messages */

/* Start sending messages START_DELAY secs after we start so that routing can
 * converge */
#define START_DELAY    300  //300

#ifndef PERIOD
#define PERIOD  30  //30  
#endif

static struct uip_udp_conn * ucast_conn;

static uip_ipaddr_t server_ipaddr;/*server global addr*/
//static struct uip_udp_conn * mcast_recv;
static uint16_t seq_id;
static char buf[MAX_PAYLOAD_LEN];

static uint8_t  p;

#define NODE_NUM  12
#define NODE_MAX  16   

struct  upcount_array {
  uint16_t  addr;
  uint16_t  upcount;
};

static struct upcount_array node_array[NODE_MAX];
 
static struct etimer et;

/*---------------------------------------------------------------------------*/
PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process,&collect_common_process);
/*---------------------------------#define RANDWAIT 300------------------------------------------*/
void
collect_common_set_sink(void)
{
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_print(void)
{
  printf("I am sink!\n");
}
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  /* Server never sends */
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_init(void)
{
#if CONTIKI_TARGET_Z1
  uart0_set_input(serial_line_input_byte);
#else
  uart1_set_input(serial_line_input_byte);
#endif
  serial_line_init();

  PRINTF("I am sink!\n");
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  uint8_t *appdata;
  linkaddr_t sender;
  uint8_t seqno;
  uint8_t hops;
  uint16_t sender_addr;
  uint16_t count ;
  uint16_t duplicate_count;
  uint8_t i;
  uint16_t curr_rank =0;
  uint8_t routing_set =0;
  uint16_t  paddr;

  if(uip_newdata()) {
    appdata = (uint8_t *)uip_appdata;
    sender.u8[0] = (UIP_IP_BUF->srcipaddr).u8[15];
    sender.u8[1] = (UIP_IP_BUF->srcipaddr).u8[14];  

 //  printf(" data_len : %u  \n" , uip_datalen());
    sender_addr = sender.u8[0] + (sender.u8[1] << 8);
  if( uip_datalen() < 12){ 
     memcpy(&count, ((uint16_t *)(uip_appdata)), sizeof(count));       
     memcpy(&duplicate_count, ((uint16_t *)(uip_appdata+2)), sizeof(duplicate_count));
     memcpy(&curr_rank, ((uint16_t *)(uip_appdata+4)), sizeof(curr_rank));
     memcpy(&paddr, ((uint16_t *)(uip_appdata+6)), sizeof(paddr));

 #if UIP_CONF_IPV6_ORPL_BITMAP   
     memcpy(&routing_set, ((uint8_t *)(uip_appdata+8)), sizeof(routing_set));
 #endif

    for (i=0; i< NODE_MAX ;i++)
    {
      if(node_array[i].addr == 0xffff )
      {
         node_array[i].addr = sender_addr ;
         node_array[i].upcount ++;
         break; 
      }else if( sender_addr == node_array[i].addr){
         node_array[i].upcount ++;
         break; 
      }       
    }
   
    printf("addr: %02x,node receive unicast: %u ,node duplicate count: %u,sink receive up: %u\n", 
      sender_addr, uip_htons(count),duplicate_count,node_array[i].upcount );
    printf("addr: %02x,parent addr: %02x,node rank: %u,node routing_set: %02x\n", 
      sender_addr,paddr,curr_rank,routing_set);
    
    struct routing_set_s *rs= orpl_routing_set_get_active();
    printf("root routing_set:%02x\n",rs->u8[0]);
    return ;
  }
  
   seqno = *appdata;
   hops = uip_ds6_if.cur_hop_limit - UIP_IP_BUF->ttl + 1;
   collect_common_recv(&sender, seqno, hops,
                        appdata + 2, uip_datalen() - 2);
  }
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*****************************************************************************/

static void
ucast_send(void)
{
  uint16_t id;
  uip_ipaddr_t ipaddr;
  uip_ip6addr(&ipaddr,0xaaaa,0,0,0,0x0012,0x7400,0x0001,node_array[p].addr);

 // PRINT6ADDR(&ipaddr);
  id = uip_htons(seq_id);
  memset(buf, 0, MAX_PAYLOAD_LEN);
  memcpy(buf, &id, sizeof(seq_id));
  printf("id : %d \n" , uip_htons(id));
  PRINTF("Send to: ");
  
  PRINT6ADDR(&ipaddr);
  //PRINT6ADDR(&ucast_conn->ripaddr);
  PRINTF(" Remote Port %u,", uip_ntohs(ucast_conn->rport));
  PRINTF(" (msg=0x%04x)", uip_ntohs(*((uint16_t *)buf)));
  PRINTF(" %u bytes\n", sizeof(id));
 
 // printf("seq_id : %u \n",seq_id);

  uip_udp_packet_sendto(ucast_conn, buf, sizeof(id),
                       &ipaddr, UIP_HTONS(UDP_PORT));
}

/*---------------------------------------------------------------------------*/
static void
set_own_addresses(void)
{
  int i;
  uint8_t state;
  rpl_dag_t *dag;
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  printf("Our IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state
        == ADDR_PREFERRED)) {
      printf("  ");
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      printf("\n");
      if(state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }

  /* Become root of a new DODAG with ID our global v6 address */
  dag = rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
  if(dag != NULL) {
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("Created a new RPL dag with ID: ");
    PRINT6ADDR(&dag->dag_id);
    PRINTF("\n");
  }
#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}
/*****************************************************************************/
static void
node_info_handler(void)
{
  
 uint8_t i=0;
 for ( i=0; i< NODE_MAX ; i++)
  {
    node_array[i].addr = 0xffff ;
    node_array[i].upcount = 0 ;
  }
  node_array[0].addr = 0x0012 ;
  node_array[1].addr = 0x0004 ;
  node_array[2].addr = 0x0007 ;
#if 1
  node_array[3].addr = 0x0006 ;
  node_array[4].addr = 0x001a ;
  node_array[5].addr = 0x0023 ;
  node_array[6].addr = 0x0013 ; 
  node_array[7].addr = 0x0025 ;
  node_array[8].addr = 0x0028 ;
  node_array[9].addr = 0x0002 ;
  node_array[10].addr = 0x0009 ;
  node_array[11].addr = 0x000b ;
#endif
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  
  PROCESS_BEGIN();

  PROCESS_PAUSE();
 
  node_info_handler();
 
  set_own_addresses();

  etimer_set(&et, START_DELAY * CLOCK_SECOND);

  PRINTF("UDP server started\n");

  print_local_addresses();
  seq_id = 1;
  p = 0;
  /* The data sink runs with a 100% duty cycle in order to ensure high
     packet reception rates. */
  NETSTACK_RDC.off(1);

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  ucast_conn = udp_new( NULL, UIP_HTONS(UDP_PORT), NULL);
  udp_bind(ucast_conn, UIP_HTONS(UCAST_SINK_UDP_PORT));

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&ucast_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(ucast_conn->lport),
         UIP_HTONS(ucast_conn->rport));
 
  while(1) {
    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {       
          ucast_send();
         //leds_toggle(LEDS_GREEN);  
          p++;
          if(p >= NODE_NUM){
            p=0;
            seq_id++; 
          }                
          etimer_set(&et, SEND_INTERVAL*PERIOD);             
        } 
      }                       
    if(ev == tcpip_event) {
      tcpip_handler();
      leds_toggle(LEDS_GREEN);
 //   printf("receive \n");
    } 
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
