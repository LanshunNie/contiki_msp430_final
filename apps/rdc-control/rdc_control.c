/**
 * \addtogroup orpl energy efficiency process
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: rdc-efficiency.c,v 1.21 2017/3/13 15:47:38 
 */

/**
 * \file
 *         A simple rdc efficiency control mechanism implet
 * \author
 *         zhangwei
 */

#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/netstack.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "net/ip/uip.h"
#include "rdc_control.h"
#include "node_function.h"
#include "task-schedule.h"
#include "sys/ctimer.h"
#include "dev/leds.h"

/*---------------------------------------------------------------------------*/
#define MCAST_RDC_UDP_PORT 3130
#define RDC_SERVER_PORT    3103
#define RDC_PAN_PORT       3102

#define RDC_CON_MAX_LEN    128
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define RDC_CONTROL_PERIOD 7

/*---------------------------------------------------------------------------*/
static struct uip_udp_conn * mcast_conn;
static char buffer[RDC_CON_MAX_LEN];
static unsigned char buffered_data_length = 0;
static struct uip_udp_conn *rdc_server_conn;
static struct task_schedule rdc_control_task;

// static struct ctimer ct;
// static int count=0;
/*---------------------------------------------------------------------------*/
static void prepare_mcast(void);
static void rdc_multicast_send(void * p);
static void tcpip_handler(void);
static void rdc_send_to_pan(void * p);
/*---------------------------------------------------------------------------*/
PROCESS(rdc_control_process, "Correct_Time process");

/*---------------------------------------------------------------------------*/
static void rdc_send_to_pan(void * p){


  char message[3];
  message[0] = RDC_CONTROL_PERIOD;
  message[1] = RDC_CONTROL_PERIOD / getNetDataTaskPeriod();
  uip_udp_packet_sendto(rdc_server_conn, message, 3,
                       get_concentrator_ipv6addr(), UIP_HTONS(RDC_PAN_PORT));

    // ctimer_set(&ct, CLOCK_SECOND*10, rdc_send_to_pan, NULL);


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
  mcast_conn = udp_new(&ipaddr, UIP_HTONS(MCAST_RDC_UDP_PORT), NULL);


}
/*---------------------------------------------------------------------------*/
static void
rdc_multicast_send(void * p)
{

  if(buffered_data_length){
    uip_udp_packet_send(mcast_conn, buffer, buffered_data_length + 1);
  }

}
/*---------------------------------------------------------------------------*/
static void 
tcpip_handler(void){

 leds_toggle(LEDS_ALL);
 // count++;
 // printf("count %d\n",count );
  if(uip_newdata()) {
    buffered_data_length = (unsigned char) ((char *)uip_appdata)[0];
    
    // printf("lenght %d , count %d\n", buffered_data_length,count);


    if(   (UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2])==0
        &&(UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1])==2)
      {

        memset(buffer, 0, RDC_CON_MAX_LEN);
        memcpy(buffer, (char *)uip_appdata, buffered_data_length + 1);
      }
  
      rdc_multicast_send(NULL);
        

  }
}
/*---------------------------------------------------------------------------*/

void 
rdc_control_init(void){
    process_start(&rdc_control_process,NULL);

}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rdc_control_process, ev, data)
{

  PROCESS_BEGIN(); 
  PROCESS_PAUSE();

  prepare_mcast();

  rdc_server_conn = udp_new(NULL, UIP_HTONS(RDC_PAN_PORT), NULL);
  if(rdc_server_conn == NULL) {
    PROCESS_EXIT();
  }
  udp_bind(rdc_server_conn, UIP_HTONS(RDC_SERVER_PORT));

  task_schedule_set(&rdc_control_task,MUST_TASK,TASK_READY,RDC_CONTROL_PERIOD,rdc_send_to_pan,NULL);
  
  // ctimer_set(&ct, CLOCK_SECOND*10, rdc_send_to_pan, NULL);


  while(1) {
    PROCESS_YIELD();
     if(ev == tcpip_event) {

      tcpip_handler();

      }

    }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
