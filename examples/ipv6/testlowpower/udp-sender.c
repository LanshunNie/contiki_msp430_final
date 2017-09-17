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
#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#else
#include "dev/uart1.h"
#endif
#include "collect-common.h"


#include "collect-view.h"



#include <stdio.h>
#include <string.h>

#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "simple-udp.h"
#include "dev/leds.h"

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#define MAX_PAYLOAD_LEN 120
#define MCAST_SINK_UDP_PORT 3001 /* Host byte order */
#define UDP_PORT 3002
// #define CMD_NUM 2
// #define LED_ON 0
// #define LED_OFF 1


static struct uip_udp_conn *sink_conn;
static uip_ipaddr_t server_ipaddr;/*server global addr*/
static struct uip_udp_conn * unicast_connection;

static uint16_t count;
static struct etimer et, et2;

//static char buf[MAX_PAYLOAD_LEN];
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#if !NETSTACK_CONF_WITH_IPV6 || !UIP_CONF_ROUTER || !UIP_CONF_IPV6_MULTICAST || !UIP_CONF_IPV6_RPL
#error "This example can not work with the current contiki configuration"
#error "Check the values of: NETSTACK_CONF_WITH_IPV6, UIP_CONF_ROUTER, UIP_CONF_IPV6_RPL"
#endif

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
/*
static void
tcpip_handler(void)
{
  if(uip_newdata()) {
    // Ignore incoming data 
  }
}  */
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
//printf (" server_ipaddr : ");
//PRINT6ADDR(&server_ipaddr);

  //printf ("   send  DATA \n");
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
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);

  /* set server address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);

}
/*
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
} */
/*
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
 */
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
} */

static void
tcpip_handler(void)
{
 
 
 //count++;
 // printf("received \n" );
  //uip_ds6_is_my_maddr(&UIP_IP_BUF->destipaddr)
  if(uip_newdata()) {

   // printf("received \n" );
    //printf("0x%08lx\n",uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata))));
    
  //  PRINTF("In: [0x%08lx], TTL %u, total %u\n",
  //  uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata))),
 //  UIP_IP_BUF->ttl, count);
   
 /*   for(i = 0; i < payload_len / 2; i++) {
    memcpy(&data, payload, sizeof(data));
    payload += sizeof(data);
    printf(" %u", data);
  }
uint8_t *appdata;
  linkaddr_t sender;
  uint8_t seqno;
  uint8_t hops;

  if(uip_newdata()) {
    appdata = (uint8_t *)uip_appdata;
    sender.u8[0] = UIP_IP_BUF->srcipaddr.u8[15];
    sender.u8[1] = UIP_IP_BUF->srcipaddr.u8[14];
    seqno = *appdata;
    hops = uip_ds6_if.cur_hop_limit - UIP_IP_BUF->ttl + 1;
    collect_common_recv(&sender, seqno, hops,
                        appdata + 2, uip_datalen() - 2);
  } */
   

 /* uint8_t *appdata;
  uint16_t seqno;
  
  printf ("  sss \n");  
    
 // appdata = (char *)uip_appdata;
    appdata = (uint8_t *)uip_appdata;
    seqno = *appdata;
    printf (" seqno xxx :  %d  \n" , seqno);
    memcpy(&seqno, appdata, sizeof(seqno));
    printf(" new seqno1 : 0x%04x \n", uip_ntohs(seqno));
    printf (" seqno zzz:  %d  \n" , uip_ntohs(seqno));

*/

   count++;
   etimer_set(&et, (30+(random_rand() % 300))* CLOCK_SECOND);  
   
    // if(count>80){
    
    // printf("total %u\n",count);

    // }
    
    //printf("%s\n","here" );
    //printf("0x%08lx\n",uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata))));
   // long  sum = uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata)));
    //printf("%ld\n",sum );
    //cmd_handler(uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata))));
   // sum%2==0? leds_on(LEDS_RED):leds_off(LEDS_RED);
    //leds_toggle(LEDS_RED);
    //PRINTF("In: [0x%08lx], TTL %u, total %u\n",
    //    uip_ntohl((unsigned long) *((uint32_t *)(uip_appdata))),
    //    UIP_IP_BUF->ttl, count);
    //printf("%s\n", "I am here!!!!!!!!!!!!!!!!");
    //printf("%X\n", UIP_IP_BUF->srcipaddr);
    
    /*
    uip_ipaddr_t rootaddr;
    uip_ip6addr(&rootaddr, 0xFE80,0,0,0,0x0212,0x7401,0x0001,0x0101);
    

    struct uip_udp_conn * response;
    response = udp_new(&(rootaddr), UIP_HTONS(MCAST_SINK_UDP_SEND_PORT), NULL);
    memset(buf,0,MAX_PAYLOAD_LEN);
    memcpy(buf,&count,sizeof(count));
    uip_udp_packet_send(response, buf, sizeof(count));
    printf("%s\n", "response");
    */
   
    //zhuanfa guocheng start
   // print_uip_addr_t(UIP_IP_BUF->srcipaddr);
   //memset(buf,0,MAX_PAYLOAD_LEN);
    //memcpy(buf,"1234444s1123111",15);
    // 接受广播数据，源地址为FE80::
    //单播返回数据将目的地址改为全局地址AAAA::
    //simple_udp_sendto(&unicast_connection, buf, strlen(buf) + 1, &server_ipaddr);
   // printf("%s", "response to");
   // print_uip_addr_t(server_ipaddr);
    //zhuanfaguocheng jiehsu 
    }
  
  return;
}
/*---------------------------------------------------------------------------*/
static uip_ds6_maddr_t *
join_mcast_group(void)
{
  uip_ipaddr_t addr;
  uip_ds6_maddr_t *rv;

  /* First, set our v6 global */
 // uip_ip6addr(&addr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
 // uip_ds6_set_addr_iid(&addr, &uip_lladdr);
  //uip_ds6_addr_add(&addr, 0, ADDR_AUTOCONF);
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
    PRINTF("Joined multicast group ");
    PRINT6ADDR(&uip_ds6_maddr_lookup(&addr)->ipaddr);
    PRINTF("\n");
  }
  return rv;
}
/*
static void set_global_address(void){
  uip_ip6addr(&server_ipaddr,0xaaaa,0,0,0,0x0212,0x7401,1,0x0101);
} */

// static void return_result(void)
// {


//   // uip_ipaddr_t rootaddr;
//   //  uip_ip6addr(&rootaddr, 0xFE80,0,0,0,0x0212,0x7401,0x0001,0x0101);
//     uint16_t sum_count;

//     char buf[10];
//     memset(buf,0,10);
//     sum_count =  uip_htons(count);
//     memcpy(buf,&(sum_count),sizeof(count));
  
//   //  printf("count : %d \n" , uip_htons(sum_count));

//    // uip_udp_packet_send(unicast_connection, buf, sizeof(count));

//  //  struct uip_udp_conn * response;
//   //  response = udp_new(&(server_ipaddr), UIP_HTONS(MCAST_SINK_UDP_PORT), NULL);
//  //  response = udp_new( NULL, UIP_HTONS(MCAST_SINK_UDP_PORT), NULL);
//   //  udp_bind(response, UIP_HTONS(UDP_PORT));
//   //  uip_udp_packet_send(response, buf, sizeof(count));

//     uip_udp_packet_sendto(unicast_connection, buf, sizeof(sum_count),
//                        &server_ipaddr, UIP_HTONS(MCAST_SINK_UDP_PORT));
//   //  printf("%s\n", "response");


// /*
//     printf("ssss \n");
//     PRINT6ADDR(&server_ipaddr);
//     uip_udp_packet_sendto(client_conn, buf, sizeof(count),
//                         &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

//     printf("     %s\n", "response"); */
    
// }

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  
  
 // static int count1;

  PROCESS_BEGIN();
 
  PROCESS_PAUSE();

 // etimer_set(&et, 60 * CLOCK_SECOND);
 // etimer_set(&et2, 10 * CLOCK_SECOND);
  PRINTF("Multicast Engine: '%s'\n", UIP_MCAST6.name);
  printf("%s\n",  UIP_MCAST6.name);
  //要想收到广播数据，必须加入同一个多播组
  if(join_mcast_group() == NULL) {
    PRINTF("Failed to join multicast group\n");
    PROCESS_EXIT();
  } 
 // set_global_address();
  count = 0;
 // count1 =0 ;
  //监听多播数据
 // sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);
 // udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));
  //servreg_hack_init();

  //注册单播的接收器
  //simple_udp_register(&unicast_connection, UDP_PORT,
   //                   NULL, UDP_PORT, receiver);

  PRINTF("Listening: ");
  PRINT6ADDR(&sink_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
        UIP_HTONS(sink_conn->lport), UIP_HTONS(sink_conn->rport));
  //内核事件处理tcpip数据事件

  set_global_address();

  PRINTF("UDP client process started\n");

  print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));


  unicast_connection = udp_new(NULL, UIP_HTONS(MCAST_SINK_UDP_PORT), NULL);
  udp_bind(unicast_connection, UIP_HTONS(UDP_PORT));

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
        UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

  NETSTACK_RADIO.off();
  
  while(1) {
    PROCESS_YIELD();
   
  if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {       
       //    etimer_set(&et, 600 * CLOCK_SECOND);   
      printf("total %u\n",count); 
      collect_common_send();
    // return_result();
      }
      else if ( data == &et2) {
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
