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
#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#else
#include "dev/uart1.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "collect-common.h"

LOWPOWERMODE
#include "collect-view.h"

#include "net/ipv6/multicast/uip-mcast6.h"
#include "simple-udp.h"
#include "dev/leds.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

static struct uip_udp_conn *server_conn;


#define MAX_PAYLOAD_LEN 120
#define MCAST_SINK_UDP_PORT 3001 /* Host byte order */
#define UDP_PORT  3002
#define SEND_INTERVAL CLOCK_SECOND /* clock ticks */
#define ITERATIONS   200 /* messages */

/* Start sending messages START_DELAY secs after we start so that routing can
 * converge */
#define START_DELAY    600

#ifndef PERIOD
#define PERIOD 18
#endif

static struct uip_udp_conn * mcast_conn;

static uip_ipaddr_t server_ipaddr;/*server global addr*/
//static struct uip_udp_conn * mcast_recv;
static char buf[MAX_PAYLOAD_LEN];
static uint16_t seq_id;
static struct simple_udp_connection unicast_connection;
#if !NETSTACK_CONF_WITH_IPV6 || !UIP_CONF_ROUTER || !UIP_CONF_IPV6_MULTICAST || !UIP_CONF_IPV6_RPL
#error "This example can not work with the current contiki configuration"
#error "Check the values of: NETSTACK_CONF_WITH_IPV6, UIP_CONF_ROUTER, UIP_CONF_IPV6_RPL"
#endif

#define NODE_MAX  20
struct  upcount_array {
  uint16_t  addr;
  uint16_t  upcount;
};

static struct upcount_array node_array[NODE_MAX];

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
  
  if(uip_newdata()) {
 
    appdata = (uint8_t *)uip_appdata;
    sender.u8[0] = UIP_IP_BUF->srcipaddr.u8[15];
    sender.u8[1] = UIP_IP_BUF->srcipaddr.u8[14];  

 //  printf(" data_len : %u  \n" , uip_datalen() );
   
  if( uip_datalen() < 3 ){

     sender_addr = sender.u8[0] + (sender.u8[1] << 8);
     
      memcpy(&count, ((uint16_t *)(uip_appdata)), sizeof(count));     
    
     uint8_t i = 0;
      for ( i=0; i< NODE_MAX ;i++)
       {
         if(node_array[i].addr == 0xffff )
         {
           node_array[i].addr = sender_addr ;
           node_array[i].upcount ++;
           break; 
          }
         else if ( sender_addr == node_array[i].addr)
         {
            node_array[i].upcount ++;
            break; 
         }       
      }
   
      printf("addr :%u , receive multicast: %u , sink_receive_up :  %u\n", 
       node_array[i].addr, uip_htons(count) , node_array[i].upcount );
      
    //  printf("ss  %u \n", uip_htons(*((uint16_t *)(uip_appdata))));
      return ;
  }

  /* if( uip_datalen() < 4)
   {
      memcpy(&count, ((uint16_t *)(uip_appdata)), sizeof(count));
     
      printf(" %u : %u \n",
         sender.u8[0] + (sender.u8[1] << 8), uip_htons(count));
      
    //  printf("ss  %u \n", uip_htons(*((uint16_t *)(uip_appdata))));
      return ;
   } */
    seqno = *appdata;
    hops = uip_ds6_if.cur_hop_limit - UIP_IP_BUF->ttl + 1;
    collect_common_recv(&sender, seqno, hops,
                        appdata + 2, uip_datalen() - 2);
  }
}
/*---------------------------------------------------------------------------*/
/*
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
      // hack to make address "final" 
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
} */
/*****************************************************************************/
/*void print_uip_addr_t(uip_ipaddr_t addr){
    int i;
    for(i=0;i<16;i++){
        printf("%02X", addr.u8[i]);
        if(i == 15){
          printf("\n");
          return;
        }
        if( i % 2 == 1){
            printf(":");
        }
    }
} */
/*
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
printf("Data received on port %d from port %d with length %d\n",
         receiver_port, sender_port, datalen);
  
  printf("%s\n", "recv ACK!!!!!!!!!!!!!!!!!!!!");

}
*/

static void
multicast_send(void)
{
  uint16_t id;
  
  uip_ipaddr_t ipaddr;
  uip_ip6addr(&ipaddr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);

  id = uip_htons(seq_id);
  memset(buf, 0, MAX_PAYLOAD_LEN);
  memcpy(buf, &id, sizeof(seq_id));
  printf("id : %d \n" , uip_htons(id));
  PRINTF("Send to: ");
  PRINT6ADDR(&mcast_conn->ripaddr);
  PRINTF(" Remote Port %u,", uip_ntohs(mcast_conn->rport));
  PRINTF(" (msg=0x%04x)", uip_ntohs(*((uint16_t *)buf)));
  PRINTF(" %u bytes\n", sizeof(id));

  seq_id++;
 // printf("seq_id : %u \n",seq_id);
 //  uip_udp_packet_send(mcast_conn, buf, sizeof(id));

  uip_udp_packet_sendto(mcast_conn, buf, sizeof(id),
                       &ipaddr, UIP_HTONS(UDP_PORT));
}
/*---------------------------------------------------------------------------*/
static void
prepare_mcast(void)
{
 // uip_ipaddr_t ipaddr;

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
   //建立多播组发送连接
 // uip_ip6addr(&ipaddr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
 // mcast_conn = udp_new(&ipaddr, UIP_HTONS(MCAST_SINK_UDP_PORT), NULL);
  
  mcast_conn = udp_new( NULL, UIP_HTONS(UDP_PORT), NULL);

 // mcast_conn = udp_new(&ipaddr, UIP_HTONS(UDP_PORT), NULL);
  udp_bind(mcast_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));

  //sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  //udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));

  //mcast_recv = udp_new(NULL,UIP_HTONS(0),NULL);
  //udp_bind(mcast_recv,UIP_HTONS(MCAST_SINK_UDP_RECV_PORT));

}
/*---------------------------------------------------------------------------*/
static void
set_own_addresses(void)
{
  int i;
  uint8_t state;
  rpl_dag_t *dag;
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
 // uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
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
#elif 1
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  //uip_ipaddr_t ipaddr;
 // struct uip_ds6_addr *root_if;
  static struct etimer et;
  
  PROCESS_BEGIN();

  PROCESS_PAUSE();
 
   uint8_t i=0;
 for ( i=0; i< NODE_MAX ; i++)
  {
    node_array[i].addr = 0xffff ;
    node_array[i].upcount = 0 ;
  }

  set_own_addresses();

  //printf("hello wolrd!@!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
  prepare_mcast();
  seq_id=0;
 /* simple_udp_register(&unicast_connection, UDP_PORT,
                      NULL, UDP_PORT, receiver);  */

  etimer_set(&et, START_DELAY * CLOCK_SECOND);
  //SENSORS_ACTIVATE(button_sensor);

  PRINTF("UDP server started\n");


  //print_local_addresses();

  /* The data sink runs with a 100% duty cycle in order to ensure high
     packet reception rates. */
  NETSTACK_RDC.off(1);

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

//  PRINTF("Created a server connection with remote address ");
 // PRINT6ADDR(&server_conn->ripaddr);
 // PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
  //       UIP_HTONS(server_conn->rport));

  while(1) {
    PROCESS_YIELD();

   if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {       
        if(seq_id >= ITERATIONS) {
           etimer_stop(&et);
           printf("end \n");
          }else {
            multicast_send();
            leds_toggle(LEDS_GREEN);
            etimer_set(&et, SEND_INTERVAL*PERIOD *20);
          }      
      }
  }
    
    if(ev == tcpip_event) {
    // printf("receive111 \n");
      tcpip_handler();
 //     printf("receive \n");
    } /*else if (ev == sensors_event && data == &button_sensor) {
      PRINTF("Initiaing global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    } */
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
