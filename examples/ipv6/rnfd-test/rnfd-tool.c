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
 *         rnfd-tool
 * \author
 *         @yang
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/rpl/rpl.h"
#include "dev/leds.h"
#include "net/netstack.h"
 
#include "simple-udp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "net/ipv6/multicast/uip-mcast6.h"
 
#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define MCAST_SINK_UDP_PORT 3001

/* Start sending messages START_DELAY secs after we start so that routing can
 * converge */
#define START_DELAY  5

#define BASE_TIME  (30 * CLOCK_SECOND)      //120

static struct uip_udp_conn *server_conn;
static struct uip_udp_conn * mcast_conn;

#define REPORT_CACHE      0xF1
#define REPORT_NONCACHE   0xF2
#define TIMED_CACHE       0xF3
#define TS_CACHE          0xF4
#define RNFD_STRAT        0xF5
/*--------------------------------------------------------*/
PROCESS(rnfd_tool, "RNFD tool");
AUTOSTART_PROCESSES(&rnfd_tool);
/*---------------------------------------------------------------------------*/
static void
multicast_send(void)
{
  uint8_t buf[10];
  memset(buf, 0, sizeof(buf));
  int pos = 0;
 
  uint8_t type = RNFD_STRAT;
  uint8_t data = 1;

  memcpy(buf, &type, sizeof(type));
  pos += sizeof(type);

  memcpy(buf+1, &data, sizeof(data));
  pos += sizeof(data);

  printf("multicast send type:%02x\n",type);
  uip_udp_packet_send(mcast_conn, buf, pos);
}
/*---------------------------------------------------------------------------*/
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
static void
tcpip_handler(void)
{
  if(uip_newdata()) {
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rnfd_tool, ev, data)
{
  static struct etimer et;
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  prepare_mcast();

  etimer_set(&et, START_DELAY * CLOCK_SECOND);
  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(server_conn == NULL) {
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  while(1) {
    PROCESS_YIELD();
    
    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {       
        multicast_send();
        leds_toggle(LEDS_GREEN);
        etimer_set(&et, BASE_TIME);        
      }
    }

    if(ev == tcpip_event) {
      tcpip_handler();
    }   
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


