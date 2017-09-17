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
//#include "dev/uart1.h"
//#include "dev/uart0.h"
 #if HW_CONF_WITH_UART1
#include "dev/uart1.h"
#else
#include "dev/uart0.h"
#endif
#include "collect-common.h"
#include "collect-view.h"
#include "dev/leds.h"
#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#include <stdio.h>
#include <string.h>
#include "lib/ringbuf.h"
#include "simple-udp.h"
#include "util.h"
#include "node-id.h"
#include "sys/ctimer.h"

#include "task-schedule.h"

#define UIP_MCAST6_CONF_ENGINE UIP_MCAST6_ENGINE_SCF
#define MAX_BUFFER_LENGTH 127
//static char data_buffer[MAX_BUFFER_LENGTH];
//port area
#define UDP_CLIENT_PORT 8775//
#define UDP_SERVER_PORT 5688//
#define UDP_SERVER_DATA_PORT 8765
#define MCAST_SINK_UDP_PORT 3001 
#define UDP_SERVER_UNICAST_PORT 5656
//所有的指令
#define CTL_READ_DATA       '1' //读表指令
#define CTL_ACK_READ_DATA   '2' //包含ACK的读表指令
#define UNICAST_ACK_DATA    '3' //danbo
#define UNICAST_CONFIG_PERIOD   '4' //unicast config period
#define MCAST_CONFIG_PERIOD   '5' //mcast config period
#define MCAST_CONFIG_HEART  '6'
#define HERAT_BODY        'h'
#define state_ready 0
#define state_ack 1
#define DEBUG DEBUG_PRINT
#define BASE_PERIOD 5* CLOCK_SECOND // /config time
#define PERIOD 5//5 30
#define SEND_INTERVAL   (PERIOD * CLOCK_SECOND) // base time
#define SEND_TIME   (random_rand() % (BASE_PERIOD))
//#define SEND_BASE 
static struct etimer report_period;
static struct etimer heart_period;
#include "net/ip/uip-debug.h"
#define MAX_PAYLOAD_LEN   255
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *client_conn_data;
static struct uip_udp_conn *client_conn_unicast_data;

static uip_ipaddr_t server_ipaddr;
static struct uip_udp_conn *sink_conn;

static char global_type;
static char ack_buffer[MAX_PAYLOAD_LEN];
static char con_buffer[MAX_PAYLOAD_LEN];
static unsigned char appdata_length;
//static unsigned char data_length;
static int data_length ;
static bool uploadFlag = false;
static int n_id;
static unsigned int global_period;
static unsigned int global_base_sec;
static unsigned int heart_sec;
static int cur_State = state_ready; 

//static char recv_buffer[MAX_PAYLOAD_LEN];
//static int node_id;

static struct task_schedule read_cmd_ts;


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
   // PRINTF("Preferred parent: ");
    PRINT6ADDR(rpl_get_parent_ipaddr(dag->preferred_parent));
   // PRINTF("\n");
  }
  for(r = uip_ds6_route_head();
      r != NULL;
      r = uip_ds6_route_next(r)) {
    PRINT6ADDR(&r->ipaddr);
  }
 // PRINTF("---\n");
}

// static void sendCmd(char* cmd,int cmd_length){
//   int i;

//   for(i=0;i< cmd_length;i++){
//     putchar((unsigned char )cmd[i]);
//   }
// }
static void sendCmd(void * p){
    int i;
    char *appdata = (char *)malloc((appdata_length +1) *sizeof(char));
    memcpy(appdata,(char *)uip_appdata + 1,appdata_length);
    appdata[appdata_length] = '\0';
    //int seconds;
    char type = appdata[0];
   // data_length =0;
   // memset(ack_buffer,0,MAX_PAYLOAD_LEN);

    char* cmd = (char *)(appdata + 1);
    int cmd_length = appdata_length -1;
     global_type = type;
     for(i=0;i< cmd_length;i++){
          putchar((unsigned char )cmd[i]);
     }
    free(appdata);
}
// static void
// send_packet(void *ptr)
// {
  
//   uip_udp_packet_sendto(client_conn_data, "h", 1,
//                       &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
//   //ctimer_set(&heart_back, 10*CLOCK_SECOND, send_packet, NULL);
// }

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {

    appdata_length = (unsigned char)((char *)uip_appdata)[0];
    char *appdata = (char *)malloc((appdata_length +1) *sizeof(char));
    memcpy(appdata,(char *)uip_appdata + 1,appdata_length);
    appdata[appdata_length] = '\0';
    //int seconds;
    char type = appdata[0];
    unsigned char ctl_content_length = 0;
   // data_length =0;
   // memset(ack_buffer,0,MAX_PAYLOAD_LEN);
    global_type = type;
    switch(type){
      case CTL_READ_DATA://duobo chun zhiling
        // sendCmd((char *)(appdata + 1),appdata_length -1); //du fa

        task_schedule_set(&read_cmd_ts,MUST_TASK,TASK_READY,sendCmd,NULL);

        // memset(ack_buffer,0,MAX_PAYLOAD_LEN);
        // memcpy(ack_buffer,appdata,appdata_length);
        //global_period = (random_rand() % (global_base_sec))+SEND_INTERVAL;
       // etimer_set(&report_period,global_period);
        //seconds =  BASE_PERIOD + SEND_TIME;
        //etimer_set(&ack_period,seconds);
       // printf("global_period %u\n",global_period);
        //uploadFlag = true;
      break;
      case CTL_ACK_READ_DATA:
        ctl_content_length = (unsigned char) appdata[1];
        if(reupload_by_acks((appdata + 2 + ctl_content_length),appdata_length - 2 - ctl_content_length ,n_id)){
            
            // sendCmd((char *)(appdata + 2),ctl_content_length);
          sendCmd(NULL);
            // memset(ack_buffer,0,MAX_PAYLOAD_LEN);
            // ack_buffer[0] = CTL_ACK_READ_DATA;
            // memcpy(ack_buffer + 1,appdata + 2,ctl_content_length);
            // appdata_length = ctl_content_length;
            // global_period = (random_rand() %(global_base_sec))+SEND_INTERVAL;
            // etimer_set(&report_period,global_period);
            // uploadFlag = true;
            //read_data((appdata + 2),ctl_content_length);
          }
      break;
      case UNICAST_ACK_DATA://
        // sendCmd((char *)(appdata+1),appdata_length-1);
                sendCmd(NULL);

        // memset(ack_buffer,0,MAX_PAYLOAD_LEN);
        // memcpy(ack_buffer,appdata,appdata_length);
        // etimer_set(&report_period,global_period);
        // uploadFlag = true;
      break;
      case MCAST_CONFIG_PERIOD:
      	// printf("appdata %s  clock %d\n",appdata+1,CLOCK_SECOND);
         global_base_sec= atoi(appdata+1) *CLOCK_SECOND;
         global_period = (random_rand() %(global_base_sec))+SEND_INTERVAL;
        // printf("global_period_s%u\n",global_period );
         con_buffer[0] = MCAST_CONFIG_PERIOD;
         //seconds =  BASE_PERIOD + SEND_TIME;
         etimer_set(&report_period,global_period);
         cur_State = state_ack;
         uploadFlag = true;
     break;
     case UNICAST_CONFIG_PERIOD://config information
         //ack_buffer[0] = UNICAST_CONFIG_PERIOD;
          //memcpy(ack_buffer,appdata,appdata_length);
        //global_period = atoi(appdata + 1) * CLOCK_SECOND +SEND_TIME;
         global_base_sec= atoi(appdata+1) *CLOCK_SECOND;
         global_period = (random_rand() %(global_base_sec))+SEND_INTERVAL;
        //con_buffer[0] = UNICAST_CONFIG_PERIOD;
        //memcpy(con_buffer + 1,appdata+1,appdata_length-1);

        uip_udp_packet_sendto(client_conn_data, "4",1,
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
        //uploadFlag = true;

      break;
      case MCAST_CONFIG_HEART://6
      	heart_sec = random_rand() %(atoi(appdata + 1) * CLOCK_SECOND);

      	etimer_set(&heart_period,heart_sec);

      break;

      }
 
    free(appdata);
  }
}

/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  static uint8_t seqno;
  struct {
    //char heart;
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
  //msg.heart =HERAT_BODY;
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
  uart0_set_input(serial_line_input_byte2);
#endif
  serial_line_init2();
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  //PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      //PRINTF("\n");
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
      //printf("\n");
    }
  }

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
  uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  rv = uip_ds6_maddr_add(&addr);

  if(rv) {
    //printf("Joined multicast group ");
    PRINT6ADDR(&uip_ds6_maddr_lookup(&addr)->ipaddr);
    //PRINTF("\n");
  }
  return rv;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  PROCESS_BEGIN();
  //printf("node--------------------------------node%d\n",uip_htons(node_id) );
  // static struct etimer periodic;
  // static struct ctimer backoff_timer;
  // static struct etimer et;
  // static struct etimer wait_et;
  if(join_mcast_group() == NULL) {
   //PRINTF("Failed to join multicast group\n");
    PROCESS_EXIT();
  }
  
  PROCESS_PAUSE();

  set_global_address();

  print_local_addresses();
  static struct ctimer heart_back;
  
  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  client_conn_data=udp_new(NULL, UIP_HTONS(UDP_SERVER_DATA_PORT), NULL);
  
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);

  client_conn_unicast_data= udp_new(NULL,UIP_HTONS(0),NULL);

  udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));

  udp_bind(client_conn_unicast_data,UIP_HTONS(UDP_SERVER_UNICAST_PORT));
  n_id = uip_htons(node_id);
  // heart_sec = 120*CLOCK_SECOND;
  // etimer_set(&heart_period,heart_sec);

 // global_period =SEND_TIME +SEND_INTERVAL;// //SEND_TIME BASE_PERIOD + 
  global_base_sec = BASE_PERIOD;//300s
  //data_length =0;
 // global_period =SEND_INTERVAL+(random_rand() %(global_base_sec));
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
     // leds_toggle(LEDS_GREEN);
      tcpip_handler();
    }
    /*触发串口事件，读取串口助手的char数据，存在data里*/
    if (ev==serial_line_event_message2)
    {
      leds_on(LEDS_GREEN);
      memset(ack_buffer,0,MAX_PAYLOAD_LEN);
      data_length =(int)((char *)data)[0];
     // data_length = (unsigned char)((char *)data)[0];
      ack_buffer[0] = global_type;
      
      memcpy(ack_buffer + 1,(char *)data+1,data_length);
      // global_period = (random_rand() % (global_base_sec))+SEND_INTERVAL;

      uip_udp_packet_sendto(client_conn_data, ack_buffer, data_length,
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));

      uploadFlag = true;
    }
    //xintiao
    // if(etimer_expired(&heart_period)){
    //   etimer_reset(&heart_period);
    //   ctimer_set(&heart_back, (random_rand() % (BASE_PERIOD)), collect_common_send, NULL);
    //   leds_toggle(LEDS_GREEN);
        
    // }
    //global_period =SEND_TIME +SEND_INTERVAL;
    if(etimer_expired(&report_period)){
      if(uploadFlag){
        if(cur_State == state_ack){
           uip_udp_packet_sendto(client_conn_data, con_buffer, 1,
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
           cur_State = state_ready;
        }else if(cur_State == state_ready){
           
           // uip_udp_packet_sendto(client_conn_data, ack_buffer, data_length,
           //            &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));

          // task_schedule_set(&task_ts,MUST_TASK,TASK_READY,node_send,NULL);


        }

        leds_off(LEDS_GREEN);
        uploadFlag = false;
      }
      
    }
}
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
