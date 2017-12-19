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
#include "task-schedule.h"
#include "node_function.h"

#include "wake-dev.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "net/ipv6/multicast/uip-mcast6.h"
 
//#define UIP_MCAST6_CONF_ENGINE UIP_MCAST6_ENGINE_SCF
#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#if LOW_LATENCY
#include "low-latency.h"
#include "low-latency-msg.h"
#endif

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define MAX_PAYLOAD_LEN 128
#define MCAST_SINK_UDP_PORT 3001
//#define temp 2016
//static struct uip_udp_conn * temp_conn;

//static struct ctimer tempct;
static struct uip_udp_conn *server_conn;
#define SEND_INTERVAL CLOCK_SECOND
 /* clock ticks */
//#define ITERATIONS  100 /* messages */
 static struct uip_udp_conn * mcast_conn;

 static uip_ipaddr_t server_ipaddr;/*server global addr*/
//static struct uip_udp_conn * mcast_recv;
//static char buff[MAX_PAYLOAD_LEN];
static char buffer[MAX_PAYLOAD_LEN];
static unsigned char buffered_data_length = 0;
///static uint16_t seq_id;

/* Start sending messages START_DELAY secs after we start so that routing can
 * converge */
#define START_DELAY  60
static uip_ipaddr_t prefix;
static uint8_t prefix_set;
/*--------------------------------------------------------*/
static void  msg_handler(char *appdata,int appdata_length);
static void root_handle_message(void*p);
static struct ctimer ct;
static struct ctimer ct1;

static int multicast_send_flag = 1;
static struct task_schedule root_task_ts;
/*--------------------------------------------------------*/

PROCESS(border_router_process, "Border router process");

#if WEBSERVER==0
/* No webserver */
AUTOSTART_PROCESSES(&border_router_process);
#elif WEBSERVER>1
/* Use an external webserver application */
#include "webserver-nogui.h"
AUTOSTART_PROCESSES(&border_router_process,&webserver_nogui_process);
#else
/* Use simple webserver with only one page for minimum footprint.
 * Multiple connections can result in interleaved tcp segments since
 * a single static buffer is used for all segments.
 */
#include "httpd-simple.h"
/* The internal webserver can provide additional information if
 * enough program flash is available.
 */
#define WEBSERVER_CONF_LOADTIME 0
#define WEBSERVER_CONF_FILESTATS 0
#define WEBSERVER_CONF_NEIGHBOR_STATUS 0
/* Adding links requires a larger RAM buffer. To avoid static allocation
 * the stack can be used for formatting; however tcp retransmissions
 * and multiple connections can result in garbled segments.
 * TODO:use PSOCk_GENERATOR_SEND and tcp state storage to fix this.
 */
#define WEBSERVER_CONF_ROUTE_LINKS 0
#if WEBSERVER_CONF_ROUTE_LINKS
#define BUF_USES_STACK 1
#endif


// static struct task_schedule send_to_pan;
// int sent_to_pan_flag = 1;


PROCESS(webserver_nogui_process, "Web server");
PROCESS_THREAD(webserver_nogui_process, ev, data)
{
  PROCESS_BEGIN();

  httpd_init();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    httpd_appcall(data);
    
  }

  PROCESS_END();
}
AUTOSTART_PROCESSES(&border_router_process,&webserver_nogui_process);

static const char *TOP = "<html><head><title>ContikiRPL</title></head><body>\n";
static const char *BOTTOM = "</body></html>\n";
#if BUF_USES_STACK
static char *bufptr, *bufend;
#define ADD(...) do {                                                   \
    bufptr += snprintf(bufptr, bufend - bufptr, __VA_ARGS__);      \
  } while(0)
#else
static char buf[256];
static int blen;
#define ADD(...) do {                                                   \
    blen += snprintf(&buf[blen], sizeof(buf) - blen, __VA_ARGS__);      \
  } while(0)
#endif

/*---------------------------------------------------------------------------*/
static void
ipaddr_add(const uip_ipaddr_t *addr)
{
  uint16_t a;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) ADD("::");
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        ADD(":");
      }
      ADD("%x", a);
    }
  }
}
/*---------------------------------------------------------------------------*/

static
PT_THREAD(generate_routes(struct httpd_state *s))
{
  static uip_ds6_route_t *r;
  static uip_ds6_nbr_t *nbr;
#if BUF_USES_STACK
  char buf[256];
#endif
#if WEBSERVER_CONF_LOADTIME
  static clock_time_t numticks;
  numticks = clock_time();
#endif

  PSOCK_BEGIN(&s->sout);

  SEND_STRING(&s->sout, TOP);
#if BUF_USES_STACK
  bufptr = buf;bufend=bufptr+sizeof(buf);
#else
  blen = 0;
#endif
  ADD("Neighbors<pre>");

  for(nbr = nbr_table_head(ds6_neighbors);
      nbr != NULL;
      nbr = nbr_table_next(ds6_neighbors, nbr)) {

#if WEBSERVER_CONF_NEIGHBOR_STATUS
#if BUF_USES_STACK
{char* j=bufptr+25;
      ipaddr_add(&nbr->ipaddr);
      while (bufptr < j) ADD(" ");
      switch (nbr->state) {
      case NBR_INCOMPLETE: ADD(" INCOMPLETE");break;
      case NBR_REACHABLE: ADD(" REACHABLE");break;
      case NBR_STALE: ADD(" STALE");break;
      case NBR_DELAY: ADD(" DELAY");break;
      case NBR_PROBE: ADD(" NBR_PROBE");break;
      }
}
#else
{uint8_t j=blen+25;
      ipaddr_add(&nbr->ipaddr);
      while (blen < j) ADD(" ");
      switch (nbr->state) {
      case NBR_INCOMPLETE: ADD(" INCOMPLETE");break;
      case NBR_REACHABLE: ADD(" REACHABLE");break;
      case NBR_STALE: ADD(" STALE");break;
      case NBR_DELAY: ADD(" DELAY");break;
      case NBR_PROBE: ADD(" NBR_PROBE");break;
      }
}
#endif
#else
      ipaddr_add(&nbr->ipaddr);
#endif

      ADD("\n");
#if BUF_USES_STACK
      if(bufptr > bufend - 45) {
        SEND_STRING(&s->sout, buf);
        bufptr = buf; bufend = bufptr + sizeof(buf);
      }
#else
      if(blen > sizeof(buf) - 45) {
        SEND_STRING(&s->sout, buf);
        blen = 0;
      }
#endif
  }
  ADD("</pre>Routes<pre>");
  SEND_STRING(&s->sout, buf);
#if BUF_USES_STACK
  bufptr = buf; bufend = bufptr + sizeof(buf);
#else
  blen = 0;
#endif

#if RPL_MOP_DOWNWARD_STORING
  for(r = uip_ds6_route_head(); r != NULL; r = uip_ds6_route_next(r)) {

#if BUF_USES_STACK
#if WEBSERVER_CONF_ROUTE_LINKS
    ADD("<a href=http://[");
    ipaddr_add(&r->ipaddr);
    ADD("]/status.shtml>");
    ipaddr_add(&r->ipaddr);
    ADD("</a>");
#else
    ipaddr_add(&r->ipaddr);
#endif
#else
#if WEBSERVER_CONF_ROUTE_LINKS
    ADD("<a href=http://[");
    ipaddr_add(&r->ipaddr);
    ADD("]/status.shtml>");
    SEND_STRING(&s->sout, buf); //TODO: why tunslip6 needs an output here, wpcapslip does not
    blen = 0;
    ipaddr_add(&r->ipaddr);
    ADD("</a>");
#else
    ipaddr_add(&r->ipaddr);
#endif
#endif
    ADD("/%u (via ", r->length);
    ipaddr_add(uip_ds6_route_nexthop(r));
    if(1 || (r->state.lifetime < 600)) {
      ADD(") %lus\n", (unsigned long)r->state.lifetime);
    } else {
      ADD(")\n");
    }
    SEND_STRING(&s->sout, buf);
#if BUF_USES_STACK
    bufptr = buf; bufend = bufptr + sizeof(buf);
#else
    blen = 0;
#endif
  }
#endif
  ADD("</pre>");

#if WEBSERVER_CONF_FILESTATS
  static uint16_t numtimes;
  ADD("<br><i>This page sent %u times</i>",++numtimes);
#endif

#if WEBSERVER_CONF_LOADTIME
  numticks = clock_time() - numticks + 1;
  ADD(" <i>(%u.%02u sec)</i>",numticks/CLOCK_SECOND,(100*(numticks%CLOCK_SECOND))/CLOCK_SECOND));
#endif

  SEND_STRING(&s->sout, buf);
  SEND_STRING(&s->sout, BOTTOM);

  PSOCK_END(&s->sout);
}

/*---------------------------------------------------------------------------*/
httpd_simple_script_t
httpd_simple_get_script(const char *name)
{
  return generate_routes;
}

#endif /* WEBSERVER */
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

static void sent_to_pan(void){
  // sent_to_pan_flag = 1;
  uip_udp_packet_sendto(server_conn, buffer + 1, buffered_data_length,
                        &server_ipaddr, UIP_HTONS(UDP_CLIENT_PORT));

}

static void
multicast_send(void * p)
{
  if(buffered_data_length){
    printf("root node multicast_send\n");
    multicast_send_flag = 1;
    uip_udp_packet_send(mcast_conn, buffer, buffered_data_length + 1);

    // sent_to_pan();
    leds_toggle(LEDS_ALL);
  
#if LOW_LATENCY 
    ctimer_set(&ct,CLOCK_SECOND*2,root_handle_message,NULL);
#endif
  }
}

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

static void root_handle_message(void*p){
  msg_handler(buffer+2,buffered_data_length-1);
}

static void
tcpip_handler(void)
{

  //  printf("sssssssssssssssssssssssssssd\n");

  if(uip_newdata()) {
    buffered_data_length = (unsigned char) ((char *)uip_appdata)[0];
    //leds_toggle(LEDS_ALL);
    // printf("DATA recv  from " );
    // printf("DATA recv  from %d" ,buffered_data_length);
    // leds_toggle(LEDS_ALL);

    uip_ip6addr(&server_ipaddr, 0xaaaa,0,0,0,0,0,0,0x0002); 
    //sprintf(buf, "Hello %d", seq_id);//封装在buffer里
    if((UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2])==0
        &&(UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1])==2)
    {

      memset(buffer, 0, MAX_PAYLOAD_LEN);
      memcpy(buffer, (char *)uip_appdata, buffered_data_length + 1);

       // multicast_send(NULL);
       // leds_toggle(LEDS_ALL);
       // sent_to_pan();
       // multicast_send_flag = 0;
        // msg_handler(buffer+2,buffered_data_length-1);
       printf("send type:%x-%x\n",(uint8_t)(*(buffer+1)),(uint8_t)(*(buffer+2)));

       // if(multicast_send_flag == 1)
        
      if((uint8_t)(*(buffer+2)) == CMD_WAKEUP) {
        printf("wake dev send\n");
    #if LOW_LATENCY 
        root_handle_message(NULL);
    #endif
      }else if(get_idle_time() >=60){      
        multicast_send(NULL);
      }

  #if LOW_LATENCY 
      if((uint8_t)(*(buffer+2)) == CMD_NETWORK_CONF_2){
          
        if(get_lowLatency_flag() ==1){
          if(get_low_latency_active_time() < 10){
            low_latency_msg_send_register(multicast_send);
          }else{
            multicast_send(NULL);
          }
        }else{
          if(get_active_flag() == 0){
            low_latency_msg_send_register(multicast_send);
          }else if(get_active_flag() == 1 && get_idle_time() <60){
            multicast_send(NULL);
          }
        }
      }
  #else
     root_handle_message(NULL);
        // ctimer_set(&ct,CLOCK_SECOND*10,root_handle_message,NULL);
  #endif
      // printf("ready send\n");
       // task_schedule_set(&root_task_ts,TEMP_TASK,TASK_READY,TASK_PERIOD_DEFAULT,multicast_send,NULL);
         // leds_toggle(LEDS_ALL);
           // multicast_send_flag = 0;
       
       
        // if(CMD_SYSTEM_MONITOR_1==(uint8_t)buffer[2]){

        //         msg_handler(buffer+2,buffered_data_length-1);
        // }else{

        // }
       
       // if(sent_to_pan_flag==1){

          // task_schedule_set(&send_to_pan,TEMP_TASK,TASK_READY,sent_to_pan,NULL);
          // sent_to_pan_flag = 0;
       // }

      
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

  // NETSTACK_RDC.off(1);//contikimac bukai rdc

  //seq_id=0;
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
   //NETSTACK_MAC.off(0);

//#if DEBUG || 1
 // print_local_addresses();
//#endif
  
  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(server_conn == NULL) {
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  //etimer_set(&et, CLOCK_SECOND);

  while(1) {
    PROCESS_YIELD();
     if(ev == tcpip_event) {

      // leds_toggle(LEDS_ALL);

      tcpip_handler();
 
      // printf("over\n");

    }
    
    // if(ev == PROCESS_EVENT_TIMER) {
    //   if(data==&et) {
    //     etimer_set(&et, CLOCK_SECOND);
      
    //   }
    // }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

// static void
// system_monitor_msg1_send(void  * p)
// {
//        uint8_t buffer_temp[SYSTEM_MONITOR_MSG_LENGTH1+2];
//        buffer_temp[0]=buffer[1];
//        buffer_temp[1]=buffer[2];

//        get_system_monitor_msg1(buffer_temp+2,SYSTEM_MONITOR_MSG_LENGTH1);

 

// uip_udp_packet_sendto(server_conn, buffer_temp, SYSTEM_MONITOR_MSG_LENGTH1+2,
//                         &server_ipaddr, UIP_HTONS(UDP_CLIENT_PORT));

// }

static void  msg_handler(char *appdata,int appdata_length)
 {
    uint8_t type=(uint8_t)appdata[0];
    switch(type)
    {
      // case CMD_SYSTEM_MONITOR_1:
      //     ctimer_set(&ct,random_rand() % (5 * CLOCK_SECOND),system_monitor_msg1_send,NULL);
      // break;


      // case CMD_NETWORK_CONF:

      //      setting_network_configuration((uint8_t*)&appdata[1],appdata_length-1);    
      // break;
      case CMD_NETWORK_CONF_2:
           leds_on(LEDS_GREEN);
           setting_network_configuration2((uint8_t*)&appdata[1],appdata_length-1);
      break;
 
      case CMD_REBOOT:
          ctimer_set(&ct1, (60 * CLOCK_SECOND),NodeReboot,NULL); 
      break;

      case CMD_RESET:
          ctimer_set(&ct,(60 * CLOCK_SECOND),NodeReset,NULL); 
          // ctimer_set(&send_ct,T_INTERVAL*n_id,node_send,NULL); 
      break;
      case CMD_WAKEUP:
    //    printf("wake node\n");
        leds_on(LEDS_GREEN);
        normalbyte_rfchannel_burn(1,WAKEUP_NODE_RFCHANNEL);
        set_init_flag(1);
#if WAKEUP_NODE       
        wake_dev_send(NULL);
#endif
         //NodeReboot(NULL);
        break;

      case CMD_CHECK_WAKEUP:
        leds_off(LEDS_GREEN);

        break;

      default:  
                    
      break;

      }
 }