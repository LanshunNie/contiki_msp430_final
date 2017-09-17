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
 
#include "net/ip/uip-debug.h"
#include <stdio.h>
#include <string.h>
#include "lib/ringbuf.h"
#include "simple-udp.h"
#include "node-id.h"
#include "sys/ctimer.h"

#include "netsynch.h"
#include "node_function.h"

#define UDP_CLIENT_PORT 8775//
#define UDP_SERVER_PORT 5688//
#define UDP_SERVER_DATA_PORT 8765
#define MCAST_SINK_UDP_PORT 3001 
#define UDP_SERVER_UNICAST_PORT 5656

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define DEBUG 0//DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define BASE_TIME  (30 * CLOCK_SECOND)    //30
#define RANDOM_TIME  (60 * CLOCK_SECOND)    // 60

static uint16_t multicast_count;
static int delay_time;
static struct ctimer ct;

static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *client_conn_data;

static uip_ipaddr_t server_ipaddr;
static struct uip_udp_conn *sink_conn;
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
static void
return_result(void * p) {      
  uint8_t msg[SYSTEM_MONITOR_MSG_LENGTH];
  get_system_monitor_msg(msg,SYSTEM_MONITOR_MSG_LENGTH);

  memcpy(msg+SYSTEM_MONITOR_MSG_LENGTH-4, &multicast_count, 2); 
  memcpy(msg+SYSTEM_MONITOR_MSG_LENGTH-2, &delay_time, 2); 
 
  uip_udp_packet_sendto(client_conn, &msg, sizeof(msg),
            &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  uint16_t seq_id;

  if(uip_newdata()) {
    uip_udp_received_data_preprocess();
    multicast_count++ ;
    
    memcpy(&seq_id, ((uint16_t *)(uip_appdata)), sizeof(seq_id)); 
    
    soft_time receive_time;
    soft_time now_time;

    memcpy(&receive_time.hour, ((uint8_t *)(uip_appdata + 2)), 1); 
    memcpy(&receive_time.minute, ((uint8_t *)(uip_appdata + 3)), 1); 
    memcpy(&receive_time.sec, ((uint8_t *)(uip_appdata + 4)), 1); 

    get_timenow(&now_time);
    delay_time = (now_time.hour-receive_time.hour) * 3600 + 
        (now_time.minute - receive_time.minute)*60 + (now_time.sec-receive_time.sec);
    
    printf("multicast receive seq_id:%u\n",seq_id);
    printf("multicast flooding delay:%d\n",delay_time);
    
    ctimer_set(&ct,BASE_TIME + (random_rand() % (RANDOM_TIME)),return_result,NULL);
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
  
  while(1) 
  {
    PROCESS_YIELD(); 
    if(ev == tcpip_event) {
      leds_toggle(LEDS_GREEN);
      tcpip_handler();
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

