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
#include "contiki-net.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "net/rpl/rpl.h"
#include "dev/serial-line.h"
#include "simple-udp.h"
#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#else
#include "dev/uart1.h"
#endif
#include "collect-common.h"
#include "collect-view.h"
#include "dev/leds.h"
 #define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#include <stdio.h>
#include <string.h>

#include "simple-udp.h"
//port area
#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688
#define UDP_SERVER_DATA_PORT 8765
#define MCAST_SINK_UDP_PORT 3001 
#define CMD_NUM 2
#define LED_ON 0
#define LED_OFF 1

#define DEBUG DEBUG_PRINT
#define PERIOD 30
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#include "net/ip/uip-debug.h"
#define MAX_PAYLOAD_LEN		128
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *client_conn_data;
static uip_ipaddr_t server_ipaddr;
static struct uip_udp_conn *sink_conn;
static uint16_t count;

static struct etimer ack_period;
static char ack_buffer[MAX_PAYLOAD_LEN];
#define 
typedef {
  int head;
  int tail;
  char buffer[MAX_PAYLOAD_LEN];
  struct etimer[] 
}Timer_queue;


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
}


void cmd_handler(long cmd){
    int t = cmd % CMD_NUM;
    switch(t){
        case LED_ON:
          leds_on(LEDS_GREEN);
          break;
        case LED_OFF:
          leds_off(LEDS_GREEN);
          break;
        default:
        printf("%s\n","error messages" );
          break;
    }
}

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  (char *)uip_appdata;
  char *appdata;
  
  //appdata = (char *)uip_appdata;
  if(uip_newdata()) {
    appdata = (char *)uip_appdata;
    memset(ack_buffer,0,MAX_PAYLOAD_LEN);
    printf("%s\n", appdata);
    sprintf(ack_buffer,appdata);
    etimer_set(&ack_period, SEND_INTERVAL);
    // while(1){
    //   if(etimer_expired(&period)){
    //     //etimer_reset(&period);
    //     leds_on(LEDS_GREEN);
    //     uip_udp_packet_sendto(client_conn_data, buff, strlen(buff),
    //                     &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
    //     PRINT6ADDR(&server_ipaddr);
    //     leds_off(LEDS_GREEN);
    //     printf("%s\n", appdata);
    //     etimer_reset(&period);
    //     return;
    //     //ctimer_set(&backoff_timer, SEND_TIME, send_packet(appdata), NULL);
    //   }
    }
  	// count++;
  	// printf("0x%08lx\n",uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata))));
  	// cmd_handler(uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata))));
    /* Ignore incoming data */
  }

static void
send_packet(void *ptr)
{
  static int seq_id;
  char buf[MAX_PAYLOAD_LEN];

  seq_id++;
  //printf("DATA send to %d 'Hello %d'\n",
        // server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
 // printf("%d\n", server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1]);
  sprintf(buf, "Hello %d from the client", seq_id);//封装在buffer里
  //sprintf(buf,appdata);
  uip_udp_packet_sendto(client_conn_data, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
  //leds_toggle(LEDS_GREEN);
  PRINT6ADDR(&server_ipaddr);
  PRINTF("\n");
}
/*---------------------------------------------------------------------------*/
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
        parent_etx = rpl_get_parent_rank((linkaddr_t *) uip_ds6_nbr_get_ll(nbr)) / 2;
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
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  /* set server address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);

}
static uip_ds6_maddr_t *
join_mcast_group(void)
{
  uip_ipaddr_t addr;
  uip_ds6_maddr_t *rv;

  /* First, set our v6 global */
  uip_ip6addr(&addr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&addr, &uip_lladdr);
  uip_ds6_addr_add(&addr, 0, ADDR_AUTOCONF);
  uint8_t state;
  int i;
   for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
      printf("\n");
    }
  }

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
  uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  rv = uip_ds6_maddr_add(&addr);

  if(rv) {
    printf("Joined multicast group ");
    PRINT6ADDR(&uip_ds6_maddr_lookup(&addr)->ipaddr);
    PRINTF("\n");
  }
  return rv;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  PROCESS_BEGIN();
  static struct etimer periodic;
  static struct ctimer backoff_timer;
  static struct etimer et;
  //printf("Multicast Engine: '%s'\n", UIP_MCAST6.name);
  //printf("%s\n", UIP_MCAST6.name);
  if(join_mcast_group() == NULL) {
   PRINTF("Failed to join multicast group\n");
    PROCESS_EXIT();
  }
  count=0;
  PROCESS_PAUSE();

  set_global_address();

  PRINTF("UDP client process started\n");

  print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  client_conn_data=udp_new(NULL, UIP_HTONS(UDP_SERVER_DATA_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));
  etimer_set(&periodic, SEND_INTERVAL);
  sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));
  printf("%s\n","listen" );

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
      printf("%s\n", "duobo");
      //leds_toggle(LEDS_GREEN);
      //etimer_set(&et, 5 * CLOCK_SECOND);
    }
    if(etimer_expired(&ack_period)){
      uip_udp_packet_sendto(client_conn_data, ack_buffer, strlen(ack_buffer),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
    }
    // else{
    //   char message[MAX_PAYLOAD_LEN];
    //   sprintf(message,"upload");
    //   //printf("%s\n", "upload");
    //   uip_udp_packet_sendto(client_conn_data, message, strlen(message),
    //                      &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
    // //printf("%s\n", "upload2");
    // }
    
    /*触发串口事件，读取串口助手的char数据，存在data里*/
    if (ev==serial_line_event_message2)
      {
        /* code */
      printf("hellworld process message \n:%s\n",(char*)data);
      char buf[MAX_PAYLOAD_LEN];
      sprintf(buf, "%s",(char*)data);//封装在buffer里

      uip_udp_packet_sendto(client_conn_data, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
      leds_toggle(LEDS_GREEN);
      PRINT6ADDR(&server_ipaddr);
      }

 //    if(etimer_expired(&periodic)) {
	//     etimer_reset(&periodic);
	//     ctimer_set(&backoff_timer, SEND_TIME, send_packet, NULL);
	//     PRINTF("Created a connection with the server ");
	//     PRINT6ADDR(&client_conn->ripaddr);
	//     PRINTF(" local/remote port %u/%u\n",
	//         UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
	// }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
