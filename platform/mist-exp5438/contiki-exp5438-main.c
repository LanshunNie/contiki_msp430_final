/*
 * Copyright (c) 2006, Swedish Institute of Computer Science
 * All rights reserved.
 *
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
 * @(#)$Id: contiki-z1-main.c,v 1.4 2010/08/26 22:08:11 nifi Exp $
 */

#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#if CONTIKI_TARGET_TRXEB1120
#include "cc1120.h"
#endif /* CONTIKI_TARGET_TRXEB1120 */
#include "dev/flash.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#if HW_CONF_WITH_UART1
#include "dev/uart1.h"
#else
#include "dev/uart0.h"
#endif
#include "dev/watchdog.h"
//#include "dev/xmem.h"
#include "lib/random.h"
//#include "lib/sensors.h"
#include "net/mac/frame802154.h"
#include "net/netstack.h"
#include "net/rime/rime.h"
#include "sys/autostart.h"

#include "wake-common.h"
 
#if CONTIKI_CONF_NETSYNCH
#include "netsynch.h"
#include "task-schedule.h"

 #include "adc.h"

  #if ROOTNODE
    #include "correct_time.h"
  #endif

   #if ROOT_WITH_ENERGY_EFFICIENCY
    #include "rdc_control.h"
  #endif

#endif
#include "node-id.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */

//zhangwei set changed for load balance
#if  WITH_ENERGY_EFFICIENCY
#include "rdc-efficiency.h"
#endif
#if LOW_LATENCY
#include "low-latency.h"
#endif

#define DEBUG 0
#include "net/ip/uip-debug.h"
/*---------------------------------------------------------------------------*/
#ifndef  RF_CHANNEL
#define  RF_CHANNEL  10
#endif
/*---------------------------------------------------------------------------*/
#if 1
static void
set_link_addr(void)
{
  linkaddr_t addr;

  memset(&addr, 0, sizeof(linkaddr_t));
#if NETSTACK_CONF_WITH_IPV6
  memcpy(addr.u8, node_mac, sizeof(addr.u8));

//  printf("Rime addr 111\n");
#else
  if(node_id == 0) {
    for(i = 0; i < sizeof(linkaddr_t); ++i) {
      addr.u8[i] = node_mac[7 - i];
    }
  } else {
    addr.u8[0] = node_id & 0xff;
    addr.u8[1] = node_id >> 8;
  }
#endif
  linkaddr_set_node_addr(&addr);
#if 0
  int i;
  printf("link addr:");
  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
    printf("%u.", addr.u8[i]);
  }
  printf("%u\n", addr.u8[i]);
#endif
}
#endif
/*---------------------------------------------------------------------------*/
static void
print_processes(struct process * const processes[])
{
  /*  const struct process * const * p = processes;*/
#if 0
  printf("Starting");
  while(*processes != NULL) {
    printf(" %s", (*processes)->name);
    processes++;
  }
#endif
  putchar('\n');
}
/*--------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
  /*
   * Initalize hardware.
   */
  msp430_cpu_init();
  clock_init();
  leds_init();

#if HW_CONF_WITH_UART1
  uart1_init(BAUD2UBR(115200)); /* Must come before first printf */
#else
  uart0_init(BAUD2UBR(115200));
#endif
 
  leds_on(LEDS_GREEN);
  /* xmem_init(); */
  
  rtimer_init();
  rtc_init();

  watchdog_init();
  // adc_initial();
  
  PRINTF(CONTIKI_VERSION_STRING "\n");
  /*  PRINTF("Compiled at %s, %s\n", __TIME__, __DATE__);*/

  /*
   * Hardware initialization done!
   */

  /* Restore node id if such has been stored in external mem */
#ifdef NODEID
  node_id = NODEID;

#ifdef BURN_NODEID
  node_id_burn(node_id);
  node_id_restore(); /* also configures node_mac[] */

  restart_count_byte_burn(0);
  normalbyte_rfchannel_burn(ABNORMAL,0);
  normalbyte_rfchannel_restore();
  uint8_t arr[]={0x05,0x10,0x5B,0xFE,0x59,0x16};
  cmd_bytes_burn(arr);
  restore_meter_cmd();
  //print_cmd_array();
#endif /* BURN_NODEID */
#else

  node_id_restore(); /* also configures node_mac[] */

   normalbyte_rfchannel_burn(0,2);
   //restart_count_byte_burn(0);
   uint8_t arr[]={0x05,0x10,0x5B,0xFE,0x59,0x16};
   cmd_bytes_burn(arr);

  normalbyte_rfchannel_restore();
   
  restart_count_byte_restore();
  restart_count++;
  restart_count_byte_burn(restart_count);
  printf("restart count %d\n",restart_count);
  
  restore_meter_cmd();
  // print_cmd_array();

#endif /* NODE_ID */

  /* for setting "hardcoded" IEEE 802.15.4 MAC addresses */
#ifdef MAC_1
  {
    uint8_t ieee[] = { MAC_1, MAC_2, MAC_3, MAC_4, MAC_5, MAC_6, MAC_7, MAC_8 };
    memcpy(node_mac, ieee, sizeof(uip_lladdr.addr));
  }
#endif

   /*
   * Initialize Contiki and our processes.
   */
  process_init();
  process_start(&etimer_process, NULL);

  ctimer_init();

  set_link_addr();

  random_init(node_id);

  NETSTACK_RADIO.init();
  // printf("%s \n",NETSTACK_RADIO);

  printf("channel set:%d\n" ,channel_byte);
  cc1120_channel_set(channel_byte);
 //  cc1120_channel_set(RF_CHANNEL);
  NETSTACK_RADIO.on();

  leds_off(LEDS_ALL);

  if(node_id > 0) {
    PRINTF("Node id %x.\n", uip_htons(node_id));
  } else {
    PRINTF("Node id not set.\n");
  }

#if NETSTACK_CONF_WITH_IPV6
  memcpy(&uip_lladdr.addr, node_mac, sizeof(uip_lladdr.addr));
  /* Setup nullmac-like MAC for 802.15.4 */

  queuebuf_init();

  netstack_init();

  /*printf("%s/%s %lu %u\n",
         NETSTACK_RDC.name,
         NETSTACK_MAC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1:
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL); */

  process_start(&tcpip_process, NULL);

#if 0
  printf("IPv6 ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
  }
#endif
  #if 0
  if(0) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, 0xfc00, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    printf("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:",
             ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n",
           ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
  }
  #endif 
#else /* NETSTACK_CONF_WITH_IPV6 */

  netstack_init();

  PRINTF("%s %lu %u\n",
         NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0? 1:
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);
#endif /* NETSTACK_CONF_WITH_IPV6 */

#if !NETSTACK_CONF_WITH_IPV6
  uart1_set_input(serial_line_input_byte);
  serial_line_init();
#endif

#if (!ROOTNODE)
#if HW_CONF_WITH_UART1
  uart1_set_input(serial_line_input_byte2);
  serial_line_init2();
#else  
  uart0_set_input(serial_line_input_byte2);
  serial_line_init2();
#endif
#endif

#if WAKEUP_NODE
  wake_common_init();
#endif
  
#if CONTIKI_CONF_NETSYNCH
 netsynch_init();

 #if  ROOTNODE  
  // netsynch_set_authority_level(0); 
 correct_time_init();
 //zhangwei set changed for load balance  
 #if ROOT_WITH_ENERGY_EFFICIENCY
 rdc_control_init();
 #endif
 #endif	
task_schedule_init();
#endif
 
//zhangwei set changed for load balance  
#if  WITH_ENERGY_EFFICIENCY
 rdc_efficiency_init();
#endif

  NETSTACK_RADIO.off();

  /*  process_start(&sensors_process, NULL);
      SENSORS_ACTIVATE(button_sensor);*/

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  watchdog_start();

  print_processes(autostart_processes);
  autostart_start(autostart_processes);

//  duty_cycle_scroller_start(CLOCK_SECOND * 2);

#if IP64_CONF_UIP_FALLBACK_INTERFACE_SLIP && WITH_SLIP
  /* Start the SLIP */
  printf("Initiating SLIP: my IP is 172.16.0.2...\n");
  slip_arch_init(0);
  {
    uip_ip4addr_t ipv4addr, netmask;

    uip_ipaddr(&ipv4addr, 172, 16, 0, 2);
    uip_ipaddr(&netmask, 255, 255, 255, 0);
    ip64_set_ipv4_address(&ipv4addr, &netmask);
  }
  uart1_set_input(slip_input_byte);
#endif /* IP64_CONF_UIP_FALLBACK_INTERFACE_SLIP */

  /*
   * This is the scheduler loop.
   */
  while(1) {
    int r;
    uint8_t low_latency_flag = 0;

    #if LOW_LATENCY
    low_latency_flag = get_lowLatency_flag();
    #endif

    do {
      /* Reset watchdog. */
      watchdog_periodic();
      r = process_run();
     // printf("process_run  \n");
    } while(r > 0);

    /*
     * Idle processing.
     */
    int s = splhigh();          /* Disable interrupts. */
    /* uart1_active is for avoiding LPM3 when still sending or receiving */
    if((process_nevents() != 0 || uart_active())&& (get_active_flag() || low_latency_flag) ){//&& get_active_flag()
      splx(s);                  /* Re-enable interrupts. */
    } else {
      // static unsigned long irq_energest = 0;

      /* Re-enable interrupts and go to sleep atomically. */
      ENERGEST_OFF(ENERGEST_TYPE_CPU);
      ENERGEST_ON(ENERGEST_TYPE_LPM);
      /* We only want to measure the processing done in IRQs when we
         are asleep, so we discard the processing time done when we
         were awake. */
      // energest_type_set(ENERGEST_TYPE_IRQ, irq_energest);
      watchdog_stop();
      _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF); /* LPM3 sleep. This
                                              statement will block
                                              until the CPU is
                                              woken up by an
                                              interrupt that sets
                                              the wake up flag. */

      /* We get the current processing time for interrupts that was
         done during the LPM and store it for next time around.  */
      dint();
      // irq_energest = energest_type_time(ENERGEST_TYPE_IRQ);
      eint();
      watchdog_start();
      ENERGEST_OFF(ENERGEST_TYPE_LPM);
      ENERGEST_ON(ENERGEST_TYPE_CPU);
    }
  }
}
/*---------------------------------------------------------------------------*/
