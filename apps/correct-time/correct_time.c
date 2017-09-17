/**
 * \addtogroup corret time
 * @{
 */


/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: corret time.c,v 1.21 2016/6/3 19:47:38 
 */

/**
 * \file
 *         A simple time correct time mechanism
 * \author
 *         zhangwei
 */
#if 1
#include "correct_time.h"
#include "contiki.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/linkaddr.h"

#include "node_function.h"
#include "net/netstack.h"

#include "netsynch.h"
 
#include <stdio.h>
#include <string.h>

#include "contiki-lib.h"
#include "contiki-net.h"
#include "simple-udp.h"

#define DEBUG 0  //DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define MAX_PAYLOAD_LEN 120

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UDP_ROOT_PORT 1026
#define UDP_TIME_PAN_PORT  1028

static struct uip_udp_conn * correct_pan_connection;

static struct correct_msg msg;

#if ROOTNODE_CORRECTTIME_TIMEOUT_REBOOT
static struct ctimer reboot_ct;
#endif
/*---------------------------------------------------------------------------*/
PROCESS(correct_time_process, "Correct_Time process");

/*---------------------------------------------------------------------------*/
static void
correct_rtc_time(soft_time cal_time)
{
  syn_update_timenow(cal_time);
}

/*---------------------------------------------------------------------------*/
static void
prepare_connect(void)
{

  correct_pan_connection = udp_new( NULL, UIP_HTONS(UDP_TIME_PAN_PORT), NULL);
  udp_bind(correct_pan_connection , UIP_HTONS(UDP_ROOT_PORT));

}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{

  // PRINTF("correct_time\n");
  //printf("sssssssd\n");

  if(uip_newdata()) {
    if(uip_ipaddr_cmp(&UIP_IP_BUF->srcipaddr, get_concentrator_ipv6addr())){
      uint8_t type=(((uint8_t *)uip_appdata)[0]);
      if(type == 0x13){
        msg.caltime.hour = (((uint8_t *)uip_appdata)[1]);

        msg.caltime.minute = (((uint8_t *)uip_appdata)[2]);

        msg.caltime.sec = (((uint8_t *)uip_appdata)[3]);

        printf("%d ,%d ,%d \n", msg.caltime.hour,msg.caltime.minute,msg.caltime.sec);

        correct_rtc_time(msg.caltime);
     
        uint8_t ack= 0x43;
        uip_udp_packet_sendto(correct_pan_connection, &ack, sizeof(ack),
            get_concentrator_ipv6addr(), UIP_HTONS(UDP_TIME_PAN_PORT));

#if ROOTNODE_CORRECTTIME_TIMEOUT_REBOOT
        ctimer_set(&reboot_ct,(720*CLOCK_SECOND),NodeReboot,NULL);   //ctimer_reset() error
#endif
      }else if(type == 0x14){
        NodeReboot(NULL);
      }
    }
#if 0
 
#endif

  }
}
/*---------------------------------------------------------------------------*/
void
correct_time_init(void)
{
#if ROOTNODE
  process_start(&correct_time_process,NULL);
         // printf("correct_time process_start\n");
  
#if ROOTNODE_CORRECTTIME_TIMEOUT_REBOOT
  ctimer_set(&reboot_ct,(720*CLOCK_SECOND),NodeReboot,NULL);
#endif
#endif
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(correct_time_process, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();
  prepare_connect();

  while(1) 
  {

    PROCESS_YIELD();

   if(ev == tcpip_event) {
       tcpip_handler();
     }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#endif