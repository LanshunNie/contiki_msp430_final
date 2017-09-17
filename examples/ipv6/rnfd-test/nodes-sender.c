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
#include "net/rpl/rpl-private.h"
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
 
#include "net/ip/uip-debug.h"
#include <stdio.h>
#include <string.h>
#include "lib/ringbuf.h"
#include "simple-udp.h"
#include "node-id.h"
#include "sys/ctimer.h"

#include "task-schedule.h"
#include "netsynch.h"
#include "node_function.h"
#include "rnfd.h"

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688
#define UDP_SERVER_DATA_PORT 8765
#define MCAST_SINK_UDP_PORT 3001 
#define UDP_SERVER_UNICAST_PORT 5656

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define DEBUG 0//DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define BASE_TIME  (30 * CLOCK_SECOND)    //30
#define RANDOM_TIME  (60 * CLOCK_SECOND)    // 60

#define REPORT_CACHE      0xF1
#define REPORT_NONCACHE   0xF2
#define TIMED_CACHE       0xF3
#define TS_CACHE          0xF4
#define RNFD_STRAT        0xF5

#define DURATION_TIME     (2*60 * CLOCK_SECOND) 

#define CACHE_MAX         20
#define CACHE_DATALEN     40
static uint8_t cache_array[CACHE_MAX][CACHE_DATALEN];
static uint8_t cache_pos;
static uint8_t report_pos;

static uint8_t rnfd_type;
static struct ctimer periodic_ts_ct;

static struct task_schedule read_data_ts;
static struct task_schedule heart_ts;

static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *client_conn_data;

static uip_ipaddr_t server_ipaddr;
static struct uip_udp_conn *sink_conn;
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
static  char msg_da[]= {0x01,0x80,0x68,0x36,0x36,0x68,0x08,0x02,0x72,0x00,0x00,0x00,0x00,0x8F,0x41,0x03,0x22,0x02,0x00,0x00,0x00,0x0F,0x07,0x00,0xF1,0x08,0x04,0x23,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x35,0x23,0x10,0x19,0x08,0x23,0x0A,0x15,0x20,0x05,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,
    0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,0x00,0x02,0x00,0x00,0x00,0x00,0xFF,0xFF}; 
/*---------------------------------------------------------------------------*/
static void send_read_meter_data(void *p)
{
  msg_da[2]=netsynch_authority_level();

  uip_udp_packet_sendto(client_conn_data, msg_da, sizeof(msg_da),
        &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
}
/*---------------------------------------------------------------------------*/  
static int 
generate_cache(uint8_t cache[]){
	int pos = 0;
	uint64_t transmit,listen;
	//uint64_t cpu,lpm;
  rpl_rank_t rank =0;

  rpl_dag_t *dag = rpl_get_any_dag();
  if(dag != NULL){
    rank = dag->rank;
  }

  memcpy(cache+pos,&rank, sizeof(rank));
  pos += sizeof(rank);

	memcpy(cache+pos,&uip_stat.ip.sent, sizeof(uip_stat.ip.sent));
  pos += sizeof(uip_stat.ip.sent);
  memcpy(cache+pos,&uip_stat.ip.recv, sizeof(uip_stat.ip.recv));
  pos += sizeof(uip_stat.ip.recv);
  memcpy(cache+pos,&uip_stat.ip.forwarded, sizeof(uip_stat.ip.forwarded));
  pos += sizeof(uip_stat.ip.forwarded);
 
  memcpy(cache+pos,&uip_stat.udp.sent, sizeof(uip_stat.udp.sent));
	pos += sizeof(uip_stat.udp.sent);
	memcpy(cache+pos,&uip_stat.udp.recv, sizeof(uip_stat.udp.recv));
	pos += sizeof(uip_stat.udp.recv);

	memcpy(cache+pos,&uip_stat.icmp.sent, sizeof(uip_stat.icmp.sent));
	pos += sizeof(uip_stat.icmp.sent);
	memcpy(cache+pos,&uip_stat.icmp.recv, sizeof(uip_stat.icmp.recv));
	pos += sizeof(uip_stat.icmp.recv);
	
	memcpy(cache+pos,&rpl_stats.local_repairs, sizeof(rpl_stats.local_repairs));
	pos += sizeof(rpl_stats.local_repairs);
	memcpy(cache+pos,&rpl_stats.resets, sizeof(rpl_stats.resets));
	pos += sizeof(rpl_stats.resets);
	memcpy(cache+pos,&rpl_stats.parent_switch, sizeof(rpl_stats.parent_switch));
	pos += sizeof(rpl_stats.parent_switch);

	energest_flush();
	//cpu      = energest_type_time(ENERGEST_TYPE_CPU)      ;
  	//lpm      = energest_type_time(ENERGEST_TYPE_LPM)      ;
  transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  listen   = energest_type_time(ENERGEST_TYPE_LISTEN);

  memcpy(cache+pos,&transmit, sizeof(transmit));
	pos += sizeof(transmit);
	memcpy(cache+pos,&listen, sizeof(listen));
	pos += sizeof(listen);

	return pos;
}
/*---------------------------------------------------------------------------*/
static void
send_data(uint8_t *buf,uint8_t pos){
  uint8_t msg[SYSTEM_MONITOR_MSG_LENGTH];

  get_system_monitor_msg(msg,SYSTEM_MONITOR_MSG_LENGTH);
  memcpy(msg+5, buf,pos);

  uip_udp_packet_sendto(client_conn, msg, sizeof(msg),
        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}

/*---------------------------------------------------------------------------*/
static void 
rnfd_data_send(void *p){
  uint8_t buf[40];
  int pos = 0;
  int data_len = 0;
  uint8_t rnfd_data[40];
  data_len = generate_cache(rnfd_data);

  if(rnfd_type == REPORT_CACHE){
  	if(report_pos < cache_pos){
  	  buf[0] = REPORT_CACHE;
  	  buf[1] = report_pos;
  	  pos += 2;
  	  memcpy(buf+pos,cache_array[report_pos],data_len);
  	  pos += data_len;

      report_pos++;
      ctimer_set(&periodic_ts_ct,BASE_TIME + (random_rand() % (RANDOM_TIME)),rnfd_data_send,NULL);
      send_data(buf,pos);
    }
  }else if(rnfd_type == REPORT_NONCACHE){
  	buf[0] = REPORT_NONCACHE;
  	pos++;
	  memcpy(buf+pos,rnfd_data,data_len);
	  pos += data_len;

  	ctimer_set(&periodic_ts_ct,DURATION_TIME,rnfd_data_send,NULL);
    send_data(buf,pos);
  }else if(rnfd_type == TIMED_CACHE){
  	if(cache_pos < CACHE_MAX){
  	   memcpy(cache_array[cache_pos],rnfd_data,data_len);
  	   cache_pos++;
  	   ctimer_set(&periodic_ts_ct,DURATION_TIME,rnfd_data_send,NULL);
       
       buf[0] = TIMED_CACHE;
       pos++;
       memcpy(buf+pos,rnfd_data,data_len);
       pos += data_len;
       send_data(buf,pos);
  	}
  }else if(rnfd_type == TS_CACHE){
  	if(cache_pos < CACHE_MAX){
  	   memcpy(cache_array[cache_pos],rnfd_data,data_len);
  	   cache_pos++;
       
       buf[0] = TS_CACHE;
       pos++;
       memcpy(buf+pos,rnfd_data,data_len);
       pos += data_len;
       send_data(buf,pos);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
msg_type_handler(uint8_t data){
  if(rnfd_type == REPORT_CACHE){
  	report_pos = 0;
  	ctimer_set(&periodic_ts_ct,BASE_TIME + (random_rand() % (RANDOM_TIME)),rnfd_data_send,NULL);
  }else if(rnfd_type == REPORT_NONCACHE){
  	ctimer_set(&periodic_ts_ct,BASE_TIME + (random_rand() % (RANDOM_TIME)),rnfd_data_send,NULL);
  }else if(rnfd_type == TIMED_CACHE){
  	cache_pos = 0;
  	ctimer_set(&periodic_ts_ct,BASE_TIME + (random_rand() % (RANDOM_TIME)),rnfd_data_send,NULL);
  }else if(rnfd_type == TS_CACHE){
  	cache_pos = 0;
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
get_msg_type(void){
   return rnfd_type;
}
/*---------------------------------------------------------------------------*/
static void
set_msg_type(uint8_t type){
   rnfd_type = type;
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  uint8_t type,data;
  static struct ctimer ct;

  if(uip_newdata()) {
    uip_udp_received_data_preprocess();

    memcpy(&type,((uint8_t *)(uip_appdata)), sizeof(type));
    printf("type:%02x\n",type);

    if(type > 0xF0){
      memcpy(&data,((uint8_t *)(uip_appdata+1)), sizeof(data));

      if(type == RNFD_STRAT){
        set_rnfd_start_flag(1);
      }else{
        set_msg_type(type);
        msg_type_handler(data);
      }
    }else {
      memcpy(&type,((uint8_t *)(uip_appdata+2)), sizeof(type));
      if(type == CMD_REBOOT){
        printf("reboot\n");
        ctimer_set(&ct,40*CLOCK_SECOND,NodeReboot,NULL); 
      }      
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  /* set server address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0x0002);
}
/*---------------------------------------------------------------------------*/
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
static void
cache_init(){
  cache_pos = 0;
  report_pos = 0;
  rnfd_type = REPORT_NONCACHE;

  memset(cache_array,0, sizeof(cache_array));
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
  
  task_schedule_set(&heart_ts,MUST_TASK,TASK_READY,TASK_PERIOD_DEFAULT,rnfd_data_send,NULL);
  task_schedule_set(&read_data_ts,MUST_TASK,TASK_READY,TASK_PERIOD_DEFAULT,send_read_meter_data,NULL);

  //ctimer_set(&periodic_ts_ct,DURATION_TIME+(random_rand() % (10*CLOCK_SECOND)),rnfd_data_send,NULL);

  cache_init();

  while(1) 
  {
    PROCESS_YIELD(); 
    if(ev == tcpip_event) {
      //leds_toggle(LEDS_GREEN);
      tcpip_handler();
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

