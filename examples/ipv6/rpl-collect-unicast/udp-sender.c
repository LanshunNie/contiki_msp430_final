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
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/rpl/rpl.h"
#include "dev/serial-line.h"
#if HW_CONF_WITH_UART1
#include "dev/uart1.h"
#else
#include "dev/uart0.h"
#endif
#include "collect-common.h"
#include "collect-view.h"

#include <stdio.h>
#include <string.h>

#if UIP_CONF_IPV6_ORPL_BITMAP
#include "net/orpl-bitmap/orpl-bitmap.h"
#include "net/orpl-bitmap/orpl-routing-set.h"
#endif

#include "contiki-lib.h"
#include "contiki-net.h"
#include "simple-udp.h"
#include "dev/leds.h"

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#define MAX_PAYLOAD_LEN 120
#define UCAST_SINK_UDP_PORT 3001 /* Host byte order */
#define UDP_PORT 3002


//static struct uip_udp_conn *sink_conn;
static uip_ipaddr_t server_ipaddr;  /*server global addr*/
static struct uip_udp_conn * unicast_connection;

static uint16_t count;
static struct etimer et, et2;
static uint8_t init_severaddr_flag;
static uint16_t  last_count;
static uint16_t  duplicate_count;

//static char buf[MAX_PAYLOAD_LEN];
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static struct uip_udp_conn *client_conn;
//static uip_ipaddr_t server_ipaddr;
struct uip_udp_conn * response;
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process, &collect_common_process);
/*---------------------------------------------------------------------------*/
void
collect_common_set_sink(void)
{
  /* A udp client can never become sink */
}
/*---------------------------------------------------------------------------*/
static void
set_server_address(void)
{
  uip_ip6addr_t *globaladdr = NULL;
  uip_ds6_addr_t * addr_desc = uip_ds6_get_global(ADDR_PREFERRED);
  if(addr_desc != NULL) {
     globaladdr = &addr_desc->ipaddr;
#if UIP_CONF_IPV6_RPL
     rpl_dag_t *dag = rpl_get_any_dag();
     if(dag) {
        init_severaddr_flag =1;
        uip_ipaddr_copy(&server_ipaddr, globaladdr);
        memcpy(&server_ipaddr.u8[8], &dag->dag_id.u8[8], sizeof(uip_ipaddr_t) / 2);  
    }
#endif
   }
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_print(void)
{
  rpl_dag_t *dag;
  uip_ds6_route_t *r;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag->preferred_parent != NULL) {
    PRINTF("Preferred parent: ");
    PRINT6ADDR(rpl_get_parent_ipaddr(dag->preferred_parent));
    PRINTF("\n");
  }
  for(r = uip_ds6_route_head();
      r != NULL;
      r = uip_ds6_route_next(r)) {
    PRINT6ADDR(&r->ipaddr);
  }
  PRINTF("---\n");
}
/*--------------------------receive-------------------------------------------------*/
void
collect_common_send(void)
{
  static uint8_t seqno;
  struct {
    uint8_t seqno;
    uint8_t for_alignment;
    struct collect_view_data_msg msg;
  } msg;
  /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  uint16_t beacon_interval;
  rpl_parent_t *preferred_parent;
  linkaddr_t parent;
  rpl_dag_t *dag;

  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  if(init_severaddr_flag==0){
     set_server_address();
  }
  memset(&msg, 0, sizeof(msg));
  seqno++;
  if(seqno == 0) {
    /* Wrap to 128 to identify restarts */
    seqno = 128;
  }
  msg.seqno = seqno;

  linkaddr_copy(&parent, &linkaddr_null);
  parent_etx = 0;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag != NULL) {
    preferred_parent = dag->preferred_parent;
    if(preferred_parent != NULL) {
      uip_ds6_nbr_t *nbr;
      nbr = uip_ds6_nbr_lookup(rpl_get_parent_ipaddr(preferred_parent));
      if(nbr != NULL) {
        /* Use parts of the IPv6 address as the parent address, in reversed byte order. */
        parent.u8[LINKADDR_SIZE - 1] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 2];
        parent.u8[LINKADDR_SIZE - 2] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 1];
        parent_etx = rpl_get_parent_rank((uip_lladdr_t *) uip_ds6_nbr_get_ll(nbr)) / 2;
      }
    }
    rtmetric = dag->rank;
    beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = uip_ds6_nbr_num();
  } else {
    rtmetric = 0;
    beacon_interval = 0;
    num_neighbors = 0;
  }
  /* num_neighbors = collect_neighbor_list_num(&tc.neighbor_list); */
  collect_view_construct_message(&msg.msg, &parent,
                                 parent_etx, rtmetric,
                                 num_neighbors, beacon_interval);

  uip_udp_packet_sendto(client_conn, &msg, sizeof(msg),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
#if 0
  printf("server_ipaddr: ");
  PRINT6ADDR(&server_ipaddr);
  printf("\n");
#endif
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
}
/*---------------------------------------------------------------------------*/
#if 0
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      // hack to make address "final" 
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
} 
void print_uip_addr_t(uip_ipaddr_t addr){
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
#endif

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  uint16_t receive_count;
  if(uip_newdata()){
    memcpy(&receive_count,((uint16_t *)(uip_appdata)), sizeof(count)); 
    if(receive_count != last_count){
       etimer_set(&et,(1+((random_rand() % 8)))* CLOCK_SECOND);
       last_count = receive_count;
       count++;
    }else{
       duplicate_count ++;
    }   
  }
  return;
}
/*---------------------------------------------------------------------------*/
static void return_result(void)
{
  uint16_t sum_count;
  char buf[10];
  uip_ipaddr_t * parent;
  rpl_dag_t *dag;
  int pos;
  uint16_t paddr;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag == NULL) {
    printf("dag null\n");
 //   return;
  }
  parent = rpl_get_parent_ipaddr(dag->preferred_parent);
  paddr = ((parent->u8[14])<<8) + (parent->u8[15]);
 // printf("paddr :%x\n",paddr);
  pos = 0;
  memset(buf,0,10);
  sum_count =  uip_htons(count);
  rpl_rank_t curr_rank = orpl_bitmap_current_rank();
  struct routing_set_s *rs= orpl_routing_set_get_active();
  uint8_t  routing_set = rs->u8[0];
  memcpy(buf+pos,&(sum_count), sizeof(count));
  pos += sizeof(count);
  memcpy(buf+pos,&duplicate_count,2);
  pos +=2;
  memcpy(buf+pos,&curr_rank,2);
  pos +=2;
  memcpy(buf+pos,&paddr,2);
  pos +=2;
#if UIP_CONF_IPV6_ORPL_BITMAP
  memcpy(buf+pos,&routing_set,1);
  pos++;
#endif

  if(init_severaddr_flag==0){
     set_server_address();
  }
//  printf("count:%d\n",uip_htons(sum_count)); 
//  printf("return result\n");
  PRINT6ADDR(&server_ipaddr);
  PRINTF("\n");

  uip_udp_packet_sendto(unicast_connection, buf, pos,
                      &server_ipaddr, UIP_HTONS(UCAST_SINK_UDP_PORT));   
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  
  PROCESS_BEGIN();
  PROCESS_PAUSE();
  
  count = 0;
  init_severaddr_flag =0;
  duplicate_count =0;
  last_count =0;

  uip_ip6addr(&server_ipaddr, 0xaaaa,0,0,0,0x0012,0x7400,0x0001,0x0028);
 /*
  PRINTF("Listening: ");
  PRINT6ADDR(&sink_conn->ripaddr);
  PRINTF("local/remote port %u/%u\n",
        UIP_HTONS(sink_conn->lport), UIP_HTONS(sink_conn->rport));

  PRINTF("UDP client process started\n");
  print_local_addresses();
 */
 // etimer_set(&et,((random_rand() % 60))* CLOCK_SECOND);
  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  unicast_connection = udp_new(NULL, UIP_HTONS(UCAST_SINK_UDP_PORT), NULL);
  udp_bind(unicast_connection, UIP_HTONS(UDP_PORT));

  /*
  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&unicast_connection->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
        UIP_HTONS(unicast_connection->lport), UIP_HTONS(unicast_connection->rport));
  */

  while(1) {
    PROCESS_YIELD();
   
  if(ev == PROCESS_EVENT_TIMER) {
    if(data == &et) {        
     //  printf("total %u\n",count); 
       // collect_common_send();
     //  etimer_set(&et,((random_rand() % 30))* CLOCK_SECOND);
       return_result();
    }else if (data == &et2) {
      leds_off(LEDS_GREEN);
    }
   }  
   if(ev == tcpip_event) {
      tcpip_handler();
      leds_on(LEDS_GREEN);
      etimer_set(&et2, 3 * CLOCK_SECOND);     
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
