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
#include <stdlib.h>
//#include "dev/uart1.h"
//#include "dev/uart0.h"
#if HW_CONF_WITH_UART1
#include "dev/uart1.h"
#else
#include "dev/uart0.h"
#endif


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
// #include "collect-common.h"
#include "netsynch.h"

#include "collect-view.h"
#define UIP_MCAST6_CONF_ENGINE UIP_MCAST6_ENGINE_SCF
#define MAX_BUFFER_LENGTH 127

//port area
#define UDP_CLIENT_PORT 8775//
#define UDP_SERVER_PORT 5688//
#define UDP_SERVER_DATA_PORT 8765
#define MCAST_SINK_UDP_PORT 3001 
#define UDP_SERVER_UNICAST_PORT 5656
//所有的指令

#define CTL_READ_DATA       '1' //读表指令
#define CTL_ACK_READ_DATA   '2' //包含ACK的读表指令

#define state_ready 0

#define DEBUG DEBUG_PRINT


//#define SEND_BASE 

static struct etimer heart_period;
#include "net/ip/uip-debug.h"
#define MAX_PAYLOAD_LEN   255
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *client_conn_data;

static uip_ipaddr_t server_ipaddr;
static struct uip_udp_conn *sink_conn;

static char global_type;
static char ack_buffer[MAX_PAYLOAD_LEN];

static unsigned char appdata_length;

static int n_id;

static unsigned int heart_sec;

static struct task_schedule task_ts;

static struct task_schedule heart_ts;

static struct task_schedule temp_task_ts;

static void node_send(void * p);
//static char recv_buffer[MAX_PAYLOAD_LEN];
//static int node_id;
static struct ctimer ct;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
static void node_send(void * p){

      soft_time times;

      printf("node_send*****\n");

      get_timenow(&times);

       ack_buffer[0] = global_type;
       ack_buffer[1] = netsynch_authority_level();
       ack_buffer[2] = get_lastseqnum();
       ack_buffer[3] = times.hour;
       ack_buffer[4] = times.minute;
       ack_buffer[5] = times.sec;
       ack_buffer[6] = netsynch_get_offset();

       uip_udp_packet_sendto(client_conn_data, ack_buffer, 7,//appdata_length
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
      
       //task_schedule_set(&temp_send,TEMP_TASK,TASK_READY,node_send,NULL);//data
        ctimer_set(&ct,40 * CLOCK_SECOND *2 +random_rand() % (5 * CLOCK_SECOND),node_send,NULL);
}

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {

    appdata_length = (unsigned char)((char *)uip_appdata)[0];
    char *appdata = (char *)malloc((appdata_length +1) *sizeof(char));
    memcpy(appdata,(char *)uip_appdata + 1,appdata_length);
    appdata[appdata_length] = '\0';

    char type = appdata[0];
    unsigned char ctl_content_length = 0;
    memset(ack_buffer,0,MAX_PAYLOAD_LEN);
    global_type = type;

    switch(type){
      case CTL_READ_DATA://duobo chun zhiling

        memset(ack_buffer,0,MAX_PAYLOAD_LEN);
        memcpy(ack_buffer,appdata,appdata_length);

        // task_schedule_set(&task_ts,MUST_TASK,TASK_READY,node_send,NULL);//data

        task_schedule_set(&task_ts,TEMP_TASK,TASK_READY,node_send,NULL);//data

        
      break;
      case CTL_ACK_READ_DATA: //chongchuan
        ctl_content_length = (unsigned char) appdata[1];
        if(reupload_by_acks((appdata + 2 + ctl_content_length),appdata_length - 2 - ctl_content_length ,n_id)){
            

            memset(ack_buffer,0,MAX_PAYLOAD_LEN);
            ack_buffer[0] = CTL_ACK_READ_DATA;
            memcpy(ack_buffer + 1,appdata + 2,ctl_content_length);
            appdata_length = ctl_content_length;
          
            task_schedule_set(&temp_task_ts,TEMP_TASK,TASK_READY,node_send,NULL);//data

          }
      break;

      }
 
    free(appdata);
  }
}

/*---------------------------------------------------------------------------*/
static void
collect_common_send(void  * p)
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



  task_schedule_set(&task_ts,TEMP_TASK,TASK_READY,node_send,NULL);//data

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
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0x0002);

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
    }
  }

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
  uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  rv = uip_ds6_maddr_add(&addr);

  if(rv) {
    PRINT6ADDR(&uip_ds6_maddr_lookup(&addr)->ipaddr);
  }
  return rv;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  PROCESS_BEGIN();
  if(join_mcast_group() == NULL) {

    PROCESS_EXIT();
  }
  
  PROCESS_PAUSE();

  set_global_address();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  client_conn_data=udp_new(NULL, UIP_HTONS(UDP_SERVER_DATA_PORT), NULL);
  
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);

  udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));

  n_id = uip_htons(node_id);
  heart_sec =  30* CLOCK_SECOND;
  etimer_set(&heart_period,heart_sec);

  while(1) {
    PROCESS_YIELD();
    
   
    if(ev == tcpip_event) {

      tcpip_handler();
    }
   
    if(etimer_expired(&heart_period)&&ev==PROCESS_EVENT_TIMER){

       // task_schedule_set(&task_ts,MUST_TASK,TASK_READY,node_send,NULL);//data

       //task_schedule_set(&heart_ts,MUST_TASK,TASK_READY,collect_common_send,NULL);

        ctimer_set(&ct,40 * CLOCK_SECOND *2+random_rand() % (5 * CLOCK_SECOND),node_send,NULL);


       etimer_stop(&heart_period);

      
    }
}
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
