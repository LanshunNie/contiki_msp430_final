/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Implementation of the ContikiMAC power-saving radio duty cycling protocol
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

#include "contiki-conf.h"
#include "dev/leds.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "lib/random.h"
#include "net/mac/mac-sequence.h"
#include "net/mac/contikimac/contikimac.h"
#include "net/netstack.h"
#include "net/rime/rime.h"
//#include "sys/compower.h"
#include "sys/pt.h"
#include "sys/rtimer.h"
#include "clock.h"
#include <string.h>
#include "wake-common.h"
#include "wake-node.h"

#include "net/ipv6/multicast/uip-mcast6.h"
#if (UIP_MCAST6_CONF_ENGINE == UIP_MCAST6_ENGINE_FCF)

#if CONTIKI_TARGET_TRXEB1120
#include "cc1120.h"
#endif

#if UIP_CONF_IPV6_ORPL_BITMAP
#include "net/orpl-bitmap/orpl-bitmap.h"
#endif

#if !ORPL_BITMAP_DUPLICATE_DETECTION
#include "net/mac/mac-sequence.h"
#endif

#if UIP_CONF_IPV6_ORPL_BITMAP
/* We add a jitter in the ContikiMAC wakeups to avoid having the same collisions repeatedly */
#define WITH_CONTIKIMIAC_JITTER   0    //1
#endif

//zhangwei set changed for load balance
#if WITH_ENERGY_EFFICIENCY
#include "net/mac/energy-efficiency/energy-efficiency.h"
#endif
#if LOW_LATENCY
#include "low-latency.h"
#endif
static uint8_t low_latency_flag = 0;

/* TX/RX cycles are synchronized with neighbor wake periods */
#ifdef CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION
#define WITH_PHASE_OPTIMIZATION      CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION
#else /* CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION */
#define WITH_PHASE_OPTIMIZATION      1
#endif /* CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION */
/* More aggressive radio sleeping when channel is busy with other traffic */
#ifndef WITH_FAST_SLEEP
#define WITH_FAST_SLEEP              1
#endif
/* Radio does CSMA and autobackoff */
#ifndef RDC_CONF_HARDWARE_CSMA
#define RDC_CONF_HARDWARE_CSMA       0
#endif
/* Radio returns TX_OK/TX_NOACK after autoack wait */
#ifndef RDC_CONF_HARDWARE_ACK
#define RDC_CONF_HARDWARE_ACK        0
#endif
/* MCU can sleep during radio off */
#ifndef RDC_CONF_MCU_SLEEP
#define RDC_CONF_MCU_SLEEP           0
#endif

#if NETSTACK_RDC_CHANNEL_CHECK_RATE >= 64
#undef WITH_PHASE_OPTIMIZATION
#define WITH_PHASE_OPTIMIZATION 0
#endif

/* CYCLE_TIME for channel cca checks, in rtimer ticks. */
static uint32_t  CYCLE_TIME ;
static uint8_t   rdc_active_channel_check_rate = NETSTACK_RDC_CHANNEL_CHECK_RATE;
static uint8_t   rdc_inactive_channel_check_interval = NETSTACK_RDC_CHANNEL_CHECK_INTERVAL; 
static uint16_t  rdc_channel_check_interval = (NETSTACK_RDC_CHANNEL_CHECK_INTERVAL * NETSTACK_RDC_CHANNEL_CHECK_RATE);
static uint16_t  rdc_channel_check_interval_count = 0;

#if 0
#ifdef CONTIKIMAC_CONF_CYCLE_TIME
#define CYCLE_TIME (CONTIKIMAC_CONF_CYCLE_TIME)
#else
#define CYCLE_TIME (RTIMER_ARCH_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE)
#endif
#endif

/* CHANNEL_CHECK_RATE is enforced to be a power of two.
 * If RTIMER_ARCH_SECOND is not also a power of two, there will be an inexact
 * number of channel checks per second due to the truncation of CYCLE_TIME.
 * This will degrade the effectiveness of phase optimization with neighbors that
 * do not have the same truncation error.
 * Define SYNC_CYCLE_STARTS to ensure an integral number of checks per second.
 */
#if RTIMER_ARCH_SECOND & (RTIMER_ARCH_SECOND - 1)
#define SYNC_CYCLE_STARTS                    1
#endif

/* Are we currently receiving a burst? */
static int we_are_receiving_burst = 0;

/* INTER_PACKET_DEADLINE is the maximum time a receiver waits for the
   next packet of a burst when FRAME_PENDING is set. */
#ifdef CONTIKIMAC_CONF_INTER_PACKET_DEADLINE
#define INTER_PACKET_DEADLINE               CONTIKIMAC_CONF_INTER_PACKET_DEADLINE
#else
#define INTER_PACKET_DEADLINE               CLOCK_SECOND / 32
#endif

/* ContikiMAC performs periodic channel checks. Each channel check
   consists of two or more CCA checks. CCA_COUNT_MAX is the number of
   CCAs to be done for each periodic channel check. The default is
   two.*/
#ifdef CONTIKIMAC_CONF_CCA_COUNT_MAX
#define CCA_COUNT_MAX                      (CONTIKIMAC_CONF_CCA_COUNT_MAX)
#else
#define CCA_COUNT_MAX                      2
#endif

/* Before starting a transmission, Contikimac checks the availability
   of the channel with CCA_COUNT_MAX_TX consecutive CCAs */
#ifdef CONTIKIMAC_CONF_CCA_COUNT_MAX_TX
#define CCA_COUNT_MAX_TX                   (CONTIKIMAC_CONF_CCA_COUNT_MAX_TX)
#else
#define CCA_COUNT_MAX_TX                   6
#endif

/* CCA_CHECK_TIME is the time it takes to perform a CCA check. */
/* Note this may be zero. AVRs have 7612 ticks/sec, but block until cca is done */
#ifdef CONTIKIMAC_CONF_CCA_CHECK_TIME
#define CCA_CHECK_TIME                     (CONTIKIMAC_CONF_CCA_CHECK_TIME)
#else
#define CCA_CHECK_TIME                     RTIMER_ARCH_SECOND / 1000  //8192
#endif

/* CCA_SLEEP_TIME is the time between two successive CCA checks. */
/* Add 1 when rtimer ticks are coarse */
#ifdef CONTIKIMAC_CONF_CCA_SLEEP_TIME
#define CCA_SLEEP_TIME CONTIKIMAC_CONF_CCA_SLEEP_TIME
#else
#if RTIMER_ARCH_SECOND > 8000
#define CCA_SLEEP_TIME                     RTIMER_ARCH_SECOND / 2000
#else
#define CCA_SLEEP_TIME                     (RTIMER_ARCH_SECOND / 2000) + 1
#endif /* RTIMER_ARCH_SECOND > 8000 */
#endif /* CONTIKIMAC_CONF_CCA_SLEEP_TIME */

/* CHECK_TIME is the total time it takes to perform CCA_COUNT_MAX
   CCAs. */
#define CHECK_TIME                         (CCA_COUNT_MAX * (CCA_CHECK_TIME + CCA_SLEEP_TIME))

/* CHECK_TIME_TX is the total time it takes to perform CCA_COUNT_MAX_TX
   CCAs. */
#define CHECK_TIME_TX                      (CCA_COUNT_MAX_TX * (CCA_CHECK_TIME + CCA_SLEEP_TIME))

/* LISTEN_TIME_AFTER_PACKET_DETECTED is the time that we keep checking
   for activity after a potential packet has been detected by a CCA
   check. */
#ifdef CONTIKIMAC_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED
#define LISTEN_TIME_AFTER_PACKET_DETECTED  CONTIKIMAC_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED
#else
#define LISTEN_TIME_AFTER_PACKET_DETECTED  RTIMER_ARCH_SECOND / 25  // 80
#endif

/* MAX_SILENCE_PERIODS is the maximum amount of periods (a period is
   CCA_CHECK_TIME + CCA_SLEEP_TIME) that we allow to be silent before
   we turn of the radio. */
#ifdef CONTIKIMAC_CONF_MAX_SILENCE_PERIODS
#define MAX_SILENCE_PERIODS                CONTIKIMAC_CONF_MAX_SILENCE_PERIODS
#else
#define MAX_SILENCE_PERIODS                3//5
#endif

/* MAX_NONACTIVITY_PERIODS is the maximum number of periods we allow
   the radio to be turned on without any packet being received, when
   WITH_FAST_SLEEP is enabled. */
#ifdef CONTIKIMAC_CONF_MAX_NONACTIVITY_PERIODS
#define MAX_NONACTIVITY_PERIODS            CONTIKIMAC_CONF_MAX_NONACTIVITY_PERIODS
#else
#define MAX_NONACTIVITY_PERIODS            7//10
#endif

/* STROBE_TIME is the maximum amount of time a transmitted packet
   should be repeatedly transmitted as part of a transmission. */
static uint32_t  STROBE_TIME;
//#define STROBE_TIME                        (CYCLE_TIME + 2 * CHECK_TIME)

/* GUARD_TIME is the time before the expected phase of a neighbor that
   a transmitted should begin transmitting packets. */
#ifdef CONTIKIMAC_CONF_GUARD_TIME
#define GUARD_TIME                         CONTIKIMAC_CONF_GUARD_TIME
#else
#define GUARD_TIME                         10 * CHECK_TIME + CHECK_TIME_TX
#endif

/* INTER_PACKET_INTERVAL is the interval between two successive packet transmissions */
#ifdef CONTIKIMAC_CONF_INTER_PACKET_INTERVAL
#define INTER_PACKET_INTERVAL              CONTIKIMAC_CONF_INTER_PACKET_INTERVAL
#else
#define INTER_PACKET_INTERVAL              RTIMER_ARCH_SECOND / 2500
#endif

/* AFTER_ACK_DETECTECT_WAIT_TIME is the time to wait after a potential
   ACK packet has been detected until we can read it out from the
   radio. */
#ifdef CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME
#define AFTER_ACK_DETECTECT_WAIT_TIME      CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME
#else
#define AFTER_ACK_DETECTECT_WAIT_TIME      RTIMER_ARCH_SECOND / 1000  //1500
#endif

/* MAX_PHASE_STROBE_TIME is the time that we transmit repeated packets
   to a neighbor for which we have a phase lock. */
#ifdef CONTIKIMAC_CONF_MAX_PHASE_STROBE_TIME
#define MAX_PHASE_STROBE_TIME              CONTIKIMAC_CONF_MAX_PHASE_STROBE_TIME
#else
#define MAX_PHASE_STROBE_TIME              RTIMER_ARCH_SECOND / 60
#endif

#ifdef CONTIKIMAC_CONF_SEND_SW_ACK
#define CONTIKIMAC_SEND_SW_ACK CONTIKIMAC_CONF_SEND_SW_ACK
#else
#define CONTIKIMAC_SEND_SW_ACK 0
#endif

#define ACK_LEN 3

#include <stdio.h>
static struct rtimer rt;
static struct pt pt;

static volatile uint8_t contikimac_is_on = 0;
static volatile uint8_t contikimac_keep_radio_on = 0;

static volatile unsigned char we_are_sending = 0;
static volatile unsigned char radio_is_on = 0;

#define TEST_ERROR_WAKE     0
#define TEST_NOACK          0
#define TEST_FCF_PROGRAM_DELAY    0

#if TEST_ERROR_WAKE
static uint32_t cca_count = 0;
static uint32_t cca_sum = 0;
#endif

#if TEST_NOACK
static uint16_t mac_noack_count = 0;
static uint16_t mac_ack_count = 0;
//collsion may be because receive not complete ack.
static uint16_t mac_collsion_count = 0;
#endif

#if TEST_FCF_PROGRAM_DELAY
static rtimer_clock_t fcf_input_t1;
static rtimer_clock_t fcf_send_cca_before_t2;
static rtimer_clock_t fcf_send_tx_t3;
#endif

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif

#if CONTIKIMAC_CONF_COMPOWER
static struct compower_activity current_packet;
#endif /* CONTIKIMAC_CONF_COMPOWER */

#if WITH_PHASE_OPTIMIZATION

#include "net/mac/phase.h"

#endif /* WITH_PHASE_OPTIMIZATION */

#define DEFAULT_STREAM_TIME (4 * CYCLE_TIME)

#if CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT
static struct timer broadcast_rate_timer;
static int broadcast_rate_counter;
#endif /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */

/*---------------------------------------------------------------------------*/
static void
on(void)
{
  if(contikimac_is_on && radio_is_on == 0) {
    radio_is_on = 1;
    NETSTACK_RADIO.on();
  }
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
  if(contikimac_is_on && radio_is_on != 0 &&
     contikimac_keep_radio_on == 0) {
    radio_is_on = 0;
    NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static volatile rtimer_clock_t cycle_start;
static void powercycle_wrapper(struct rtimer *t, void *ptr);
static char powercycle(struct rtimer *t, void *ptr);
static void
schedule_powercycle(struct rtimer *t, rtimer_clock_t time)
{
  int r;
  rtimer_clock_t now;

  if(contikimac_is_on) {

    time += RTIMER_TIME(t);
    now = RTIMER_NOW();
    if(RTIMER_CLOCK_LT(time, now + RTIMER_GUARD_TIME)) {
      time = now + RTIMER_GUARD_TIME;
    }

    r = rtimer_set(t, time, 1, powercycle_wrapper, NULL);

    if(r != RTIMER_OK) {
      PRINTF("schedule_powercycle: could not set rtimer\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
schedule_powercycle_fixed(struct rtimer *t, rtimer_clock_t fixed_time)
{
  int r;
  rtimer_clock_t now;

  if(contikimac_is_on) {

    now = RTIMER_NOW();
    if(RTIMER_CLOCK_LT(fixed_time, now + RTIMER_GUARD_TIME)) {
      fixed_time = now + RTIMER_GUARD_TIME;
    }

    r = rtimer_set(t, fixed_time, 1, powercycle_wrapper, NULL);
    if(r != RTIMER_OK) {
      PRINTF("schedule_powercycle: could not set rtimer\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
powercycle_turn_radio_off(void)
{
#if CONTIKIMAC_CONF_COMPOWER
  uint8_t was_on = radio_is_on;
#endif /* CONTIKIMAC_CONF_COMPOWER */
  
  if(we_are_sending == 0 && we_are_receiving_burst == 0) {
    off();
#if CONTIKIMAC_CONF_COMPOWER
    if(was_on && !radio_is_on) {
      compower_accumulate(&compower_idle_activity);
    }
#endif /* CONTIKIMAC_CONF_COMPOWER */
  }
}
/*---------------------------------------------------------------------------*/
static void
powercycle_turn_radio_on(void)
{
  if(we_are_sending == 0 && we_are_receiving_burst == 0) {
    on();
  }
}
/*---------------------------------------------------------------------------*/
static void
powercycle_wrapper(struct rtimer *t, void *ptr)
{
  powercycle(t, ptr);
}
/*---------------------------------------------------------------------------*/
static char
powercycle(struct rtimer *t, void *ptr)
{
#if SYNC_CYCLE_STARTS
  static volatile rtimer_clock_t sync_cycle_start;
  static volatile uint8_t sync_cycle_phase;
#endif

  PT_BEGIN(&pt);

#if SYNC_CYCLE_STARTS
  sync_cycle_start = RTIMER_NOW();
#else
  cycle_start = RTIMER_NOW();
#endif

  while(1) {
    static uint8_t packet_seen;
    static uint8_t count;

#if SYNC_CYCLE_STARTS
    /* Compute cycle start when RTIMER_ARCH_SECOND is not a multiple
       of CHANNEL_CHECK_RATE */
    if(sync_cycle_phase++ == NETSTACK_RDC_CHANNEL_CHECK_RATE) {
      sync_cycle_phase = 0;
      sync_cycle_start += RTIMER_ARCH_SECOND;
      cycle_start = sync_cycle_start;
    } else {
#if (RTIMER_ARCH_SECOND * NETSTACK_RDC_CHANNEL_CHECK_RATE) > 65535
      cycle_start = sync_cycle_start + ((unsigned long)(sync_cycle_phase*RTIMER_ARCH_SECOND))/NETSTACK_RDC_CHANNEL_CHECK_RATE;
#else
      cycle_start = sync_cycle_start + (sync_cycle_phase*RTIMER_ARCH_SECOND)/NETSTACK_RDC_CHANNEL_CHECK_RATE;
#endif
    }
#else
    cycle_start += CYCLE_TIME;
#endif

    rdc_channel_check_interval_count ++;
    //get_init_flag()==0 ;
    #if LOW_LATENCY
    low_latency_flag = get_lowLatency_flag();
    #endif

  if(get_wake_send()== 0&&(low_latency_flag==1 || get_active_flag() || ( get_init_flag()==0 &&
     rdc_channel_check_interval_count >= rdc_channel_check_interval))){
    
   // printf("do cca \n");

    rdc_channel_check_interval_count = 0;
    packet_seen = 0;

    for(count = 0; count < CCA_COUNT_MAX; ++count) {
      if(we_are_sending == 0 && we_are_receiving_burst == 0) {
        powercycle_turn_radio_on();
        /* Check if a packet is seen in the air. If so, we keep the
             radio on for a while (LISTEN_TIME_AFTER_PACKET_DETECTED) to
             be able to receive the packet. We also continuously check
             the radio medium to make sure that we wasn't woken up by a
             false positive: a spurious radio interference that was not
             caused by an incoming packet. */
        if(NETSTACK_RADIO.channel_clear() == 0) {
          packet_seen = 1;
#if TEST_ERROR_WAKE            
          ++cca_count;
          leds_toggle(LEDS_GREEN);
#endif
          break;
        }
#if TEST_ERROR_WAKE  
       ++cca_sum;
#endif
        powercycle_turn_radio_off();
      }
      schedule_powercycle_fixed(t, RTIMER_NOW() + CCA_SLEEP_TIME);
      PT_YIELD(&pt);
    }

#if TEST_ERROR_WAKE   
    printf("cca_count:%lu , cca_sum:%lu\n",cca_count,cca_sum);
#endif

    if(packet_seen) {
      static rtimer_clock_t start;
      static uint8_t silence_periods, periods;
      start = RTIMER_NOW();

      periods = silence_periods = 0;

#if FCF_OPTIMIZE_ENERGY & (FCF_INTER_PACKET_INTERVAL > (3 * CCA_CHECK_TIME))  
    static uint8_t constant_on;
    constant_on = 0;
#endif

      while(we_are_sending == 0 && radio_is_on &&
            RTIMER_CLOCK_LT(RTIMER_NOW(),
                            (start + LISTEN_TIME_AFTER_PACKET_DETECTED))) {

        /* Check for a number of consecutive periods of
             non-activity. If we see two such periods, we turn the
             radio off. Also, if a packet has been successfully
             received (as indicated by the
             NETSTACK_RADIO.pending_packet() function), we stop
             snooping. */
#if !RDC_CONF_HARDWARE_CSMA
       /* A cca cycle will disrupt rx on some radios, e.g. mc1322x, rf230 */
       /*TODO: Modify those drivers to just return the internal RSSI when already in rx mode */
        if(NETSTACK_RADIO.channel_clear()) {
          ++silence_periods; 
          
#if FCF_OPTIMIZE_ENERGY 
         // constant_on = 1;
#endif          
        } else {
          silence_periods = 0;
        }
#endif
        ++periods;

        if(NETSTACK_RADIO.receiving_packet()) {
          silence_periods = 0;
        }
        if(silence_periods > MAX_SILENCE_PERIODS) {
          powercycle_turn_radio_off();
          break;
        }
        if(WITH_FAST_SLEEP &&
            periods > MAX_NONACTIVITY_PERIODS &&
            !(NETSTACK_RADIO.receiving_packet() ||
              NETSTACK_RADIO.pending_packet())) {
          powercycle_turn_radio_off();
          break;
        }
        if(NETSTACK_RADIO.pending_packet()) {
          break;
        }
        
        if(!packet_is_for_us()){
          powercycle_turn_radio_off();
          break;
        }        

#if FCF_OPTIMIZE_ENERGY & (FCF_INTER_PACKET_INTERVAL > (3 * CCA_CHECK_TIME))  
        if(constant_on) {
          powercycle_turn_radio_on();
          schedule_powercycle(t, CCA_CHECK_TIME + CCA_SLEEP_TIME);  
        }else {
          constant_on = 1;
          schedule_powercycle(t, FCF_INTER_PACKET_INTERVAL - 2 *CCA_CHECK_TIME);
          powercycle_turn_radio_off();
          PT_YIELD(&pt);

          schedule_powercycle(t, 2 * CCA_CHECK_TIME);
          powercycle_turn_radio_on(); 
        }
        PT_YIELD(&pt);
#else
        schedule_powercycle(t, CCA_CHECK_TIME + CCA_SLEEP_TIME);
        PT_YIELD(&pt);
#endif /* FCF_OPTIMIZE_ENERGY */
      }
      if(radio_is_on) {
        if(!(NETSTACK_RADIO.receiving_packet() ||
             NETSTACK_RADIO.pending_packet()) ||
             !RTIMER_CLOCK_LT(RTIMER_NOW(),
                 (start + LISTEN_TIME_AFTER_PACKET_DETECTED))) {
          powercycle_turn_radio_off();
        }
      }
    }
  }
    if(RTIMER_CLOCK_LT(RTIMER_NOW() - cycle_start, CYCLE_TIME - CHECK_TIME * 4)) {
      /* Schedule the next powercycle interrupt, or sleep the mcu
	 until then.  Sleeping will not exit from this interrupt, so
	 ensure an occasional wake cycle or foreground processing will
	 be blocked until a packet is detected */
#if RDC_CONF_MCU_SLEEP
      static uint8_t sleepcycle;
      if((sleepcycle++ < 16) && !we_are_sending && !radio_is_on) {
        rtimer_arch_sleep(CYCLE_TIME - (RTIMER_NOW() - cycle_start));
      } else {
        sleepcycle = 0;
        schedule_powercycle_fixed(t, CYCLE_TIME + cycle_start);
        PT_YIELD(&pt);
      }
#else
#if WITH_CONTIKIMIAC_JITTER
      schedule_powercycle(t, CYCLE_TIME - (random_rand() % (CYCLE_TIME/8)));
#else
      schedule_powercycle_fixed(t, CYCLE_TIME + cycle_start);
#endif
      PT_YIELD(&pt);
#endif
    }
  }

  PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
static int
broadcast_rate_drop(void)
{
#if CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT
  if(!timer_expired(&broadcast_rate_timer)) {
    broadcast_rate_counter++;
    if(broadcast_rate_counter < CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT) {
      return 0;
    } else {
      return 1;
    }
  } else {
    timer_set(&broadcast_rate_timer, CLOCK_SECOND);
    broadcast_rate_counter = 0;
    return 0;
  }
#else /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */
  return 0;
#endif /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */
}
/*---------------------------------------------------------------------------*/
static int
send_packet(mac_callback_t mac_callback, void *mac_callback_ptr,
	    struct rdc_buf_list *buf_list,
            int is_receiver_awake)
{
  rtimer_clock_t t0;
#if WITH_PHASE_OPTIMIZATION
  rtimer_clock_t encounter_time = 0;
#endif
  int strobes;
  uint8_t got_strobe_ack = 0;
  uint8_t is_broadcast = 0;
  uint8_t is_known_receiver = 0;
  uint8_t collisions;
  int transmit_len;
  int ret;
  uint8_t contikimac_was_on;
#if !RDC_CONF_HARDWARE_ACK
  int len;
  uint8_t seqno;
#endif
 
#if TEST_ERROR_WAKE
  return MAC_TX_OK;
#endif
  
// inactive don't send packet ,just return mac_tx_ok ;
#if TRXEB1120_CONF_LOWPOWER 
#if LOW_LATENCY
  low_latency_flag = get_lowLatency_flag();
#endif
  
 if(get_wake_send()== 0 && ((get_active_flag()== 0 ||get_idle_time()<= 4) && low_latency_flag == 0))
   return MAC_TX_OK;
#endif

  /* Exit if RDC and radio were explicitly turned off */
   if(!contikimac_is_on && !contikimac_keep_radio_on) {
    PRINTF("contikimac: radio is turned off\n");
    return MAC_TX_ERR_FATAL;
  }
 
  if(packetbuf_totlen() == 0) {
    PRINTF("contikimac: send_packet data len 0\n");
    return MAC_TX_ERR_FATAL;
  }

#if !NETSTACK_CONF_BRIDGE_MODE
  /* If NETSTACK_CONF_BRIDGE_MODE is set, assume PACKETBUF_ADDR_SENDER is already set. */
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
#endif
  if(packetbuf_holds_broadcast()) {
    is_broadcast = 1;
    PRINTDEBUG("contikimac: send broadcast\n");

    if(broadcast_rate_drop()) {
      return MAC_TX_COLLISION;
    }
  } else {

    /*
     *zhangwei set changed for load balance
     *
     *
     */
    uip_ipaddr_t unicast_ip;

    uip_ip6addr(&unicast_ip, 0xaaaa, 0, 0, 0, 0, 0, 0, 0x0002);
    #if WITH_ENERGY_EFFICIENCY
      ENERGY_EFFICIENCY_ADJUST_BEHAVIOUR(unicast_ip);
    #endif
      // printf("-----------------------\n");


#if NETSTACK_CONF_WITH_IPV6
    PRINTDEBUG("contikimac: send unicast to %02x%02x:%02x%02x:%02x%02x:%02x%02x\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[2],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[3],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[4],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[5],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[6],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[7]);
#else /* NETSTACK_CONF_WITH_IPV6 */
    PRINTDEBUG("contikimac: send unicast to %u.%u\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1]);
#endif /* NETSTACK_CONF_WITH_IPV6 */
  }

  if(!packetbuf_attr(PACKETBUF_ATTR_IS_CREATED_AND_SECURED)) {
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
    if(NETSTACK_FRAMER.create() < 0) {
      PRINTF("contikimac: framer failed\n");
      return MAC_TX_ERR_FATAL;
    }
  }
  
  transmit_len = packetbuf_totlen();
  NETSTACK_RADIO.prepare(packetbuf_hdrptr(), transmit_len);
  
  if(!is_broadcast && !is_receiver_awake) {
#if WITH_PHASE_OPTIMIZATION
    ret = phase_wait(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     CYCLE_TIME, GUARD_TIME,
                     mac_callback, mac_callback_ptr, buf_list);
    if(ret == PHASE_DEFERRED) {
      return MAC_TX_DEFERRED;
    }
    if(ret != PHASE_UNKNOWN) {
      is_known_receiver = 1;
    }
#endif /* WITH_PHASE_OPTIMIZATION */ 
  }
  

  /* By setting we_are_sending to one, we ensure that the rtimer
     powercycle interrupt do not interfere with us sending the packet. */
  we_are_sending = 1;

  /* If we have a pending packet in the radio, we should not send now,
     because we will trash the received packet. Instead, we signal
     that we have a collision, which lets the packet be received. This
     packet will be retransmitted later by the MAC protocol
     instread. */
  if(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet()) {
    we_are_sending = 0;
    PRINTF("contikimac: collision receiving %d, pending %d\n",
           NETSTACK_RADIO.receiving_packet(), NETSTACK_RADIO.pending_packet());
    return MAC_TX_COLLISION;
  }
  
  /* Switch off the radio to ensure that we didn't start sending while
     the radio was doing a channel check. */
  off();


  strobes = 0;

  /* Send a train of strobes until the receiver answers with an ACK. */
  collisions = 0;

  got_strobe_ack = 0;

  /* Set contikimac_is_on to one to allow the on() and off() functions
     to control the radio. We restore the old value of
     contikimac_is_on when we are done. */
  contikimac_was_on = contikimac_is_on;
  contikimac_is_on = 1;

#if TEST_FCF_PROGRAM_DELAY
  fcf_send_cca_before_t2 = RTIMER_NOW();
#endif

#if !RDC_CONF_HARDWARE_CSMA
    /* Check if there are any transmissions by others. */
    /* TODO: why does this give collisions before sending with the mc1322x? */
  if(is_receiver_awake == 0) {
    int i;
    for(i = 0; i < CCA_COUNT_MAX_TX; ++i) {
      t0 = RTIMER_NOW();
      on();
#if CCA_CHECK_TIME > 0
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_CHECK_TIME)) { }
#endif
      if(NETSTACK_RADIO.channel_clear() == 0) {
        collisions++;
        off();
        break;
      }
      off();

      if(is_broadcast){
        break;
      }

      t0 = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_SLEEP_TIME)) {}
    }
  }

  if(collisions > 0) {
    we_are_sending = 0;
    off();

#if ROOTNODE
    printf("contikimac: collisions before sending\n");
    //leds_toggle(LEDS_GREEN);
#endif
    contikimac_is_on = contikimac_was_on;
    return MAC_TX_COLLISION;
  }
#endif /* RDC_CONF_HARDWARE_CSMA */

#if !RDC_CONF_HARDWARE_ACK
  if(!is_broadcast) {
    /* Turn radio on to receive expected unicast ack.  Not necessary
       with hardware ack detection, and may trigger an unnecessary cca
       or rx cycle */
    on();
  }
  seqno = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
#endif
 
#if TEST_FCF_PROGRAM_DELAY
  fcf_send_tx_t3 = RTIMER_NOW();
#endif

#if TEST_FCF_PROGRAM_DELAY
  printf("fcf_t1:%u,fcf_cca_before_t2:%u,fcf_send_tx_t3:%u\n",fcf_input_t1,fcf_send_cca_before_t2,fcf_send_tx_t3);
#endif

  watchdog_periodic();
  t0 = RTIMER_NOW();

#if WAKEUP_NODE
  static uint32_t old_strobe_time = 0;
  int rdc_count =1;
  old_strobe_time =STROBE_TIME;
  if(get_wake_send()){
    STROBE_TIME = RTIMER_SECOND/4;
    rdc_count= (NETSTACK_RDC_CHANNEL_CHECK_INTERVAL+1)*4;
  }
  int j;

  //leds_on(LEDS_GREEN);
  for(j=0;j<rdc_count && got_strobe_ack == 0 && collisions == 0;j++){
    watchdog_periodic();
    t0 = RTIMER_NOW();
#endif
 
  for(strobes = 0, collisions = 0;
      got_strobe_ack == 0 && collisions == 0 && (get_wake_send()||get_active_flag() || low_latency_flag) &&
      RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + STROBE_TIME); strobes++) {
    watchdog_periodic();
    if(!is_broadcast && (is_receiver_awake || is_known_receiver) &&
       !RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + MAX_PHASE_STROBE_TIME)) {
      PRINTF("miss to %d\n", packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0]);
      break;
    }

#if !RDC_CONF_HARDWARE_ACK
    len = 0;
#endif

    {
      rtimer_clock_t wt;
#if WITH_PHASE_OPTIMIZATION
      rtimer_clock_t txtime = RTIMER_NOW();
#endif
#if RDC_CONF_HARDWARE_ACK
      int ret = NETSTACK_RADIO.transmit(transmit_len);
#else
      NETSTACK_RADIO.transmit(transmit_len);
#endif
      
#if RDC_CONF_HARDWARE_ACK
     /* For radios that block in the transmit routine and detect the
	ACK in hardware */
      if(ret == RADIO_TX_OK) {
        if(!is_broadcast) {
          got_strobe_ack = 1;
#if WITH_PHASE_OPTIMIZATION
          encounter_time = txtime;
#endif
          break;
        }
      } else if (ret == RADIO_TX_NOACK) {
      } else if (ret == RADIO_TX_COLLISION) {
          PRINTF("contikimac: collisions while sending\n");
          collisions++;
      }
      wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + INTER_PACKET_INTERVAL)) { }
#else /* RDC_CONF_HARDWARE_ACK */
     /* Wait for the ACK packet */
      wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + FCF_ACK_WAIT_TIME)) { }

      if(!is_broadcast && (NETSTACK_RADIO.receiving_packet() ||
                           NETSTACK_RADIO.pending_packet() ||
                           NETSTACK_RADIO.channel_clear() == 0)) {
        uint8_t ack_len = ACK_LEN; 
        uint8_t ackbuf[ack_len];
        wt = RTIMER_NOW();
        while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + AFTER_ACK_DETECTECT_WAIT_TIME)) { }
        
        len = NETSTACK_RADIO.read(ackbuf, ack_len);
        if(len == ack_len && seqno == ackbuf[ack_len-1]) {
          got_strobe_ack = 1;
#if WITH_PHASE_OPTIMIZATION
          encounter_time = txtime;
#endif
#if TEST_NOACK 
          mac_ack_count++;
#endif            
          break;
        } else {
          PRINTF("contikimac: collisions while sending\n");
          collisions++;
#if TEST_NOACK 
          mac_collsion_count++;
#endif
          break;
        }
      }

#if FCF_REMAIN_OPTIMIZE_ENERGY & (FCF_REMAIN_INTER_PACKET_INTERVAL > (3 * CCA_CHECK_TIME))         
      if(!is_broadcast){
        wt = RTIMER_NOW();
        off();
        while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + FCF_REMAIN_INTER_PACKET_INTERVAL - 2 * CCA_CHECK_TIME)) { }
        
        wt = RTIMER_NOW();
        on();
        while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + 2 * CCA_CHECK_TIME)) { }  
      }else {
        wt = RTIMER_NOW();
        while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + FCF_REMAIN_INTER_PACKET_INTERVAL)) { } ;
      }      
#else
      wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + FCF_REMAIN_INTER_PACKET_INTERVAL)) { } ;
#endif /* FCF_REMAIN_OPTIMIZE_ENERGY */

#endif /* RDC_CONF_HARDWARE_ACK */
    }
  }
  
#if WAKEUP_NODE
 }
  STROBE_TIME = old_strobe_time;
#endif
  //leds_off(LEDS_GREEN);

  off();

  PRINTF("contikimac: send (strobes=%u, len=%u, %s, %s), done\n", strobes,
         packetbuf_totlen(),
         got_strobe_ack ? "ack" : "no ack",
         collisions ? "collision" : "no collision");

#if CONTIKIMAC_CONF_COMPOWER
  /* Accumulate the power consumption for the packet transmission. */
  compower_accumulate(&current_packet);

  /* Convert the accumulated power consumption for the transmitted
     packet to packet attributes so that the higher levels can keep
     track of the amount of energy spent on transmitting the
     packet. */
  compower_attrconv(&current_packet);

  /* Clear the accumulated power consumption so that it is ready for
     the next packet. */
  compower_clear(&current_packet);
#endif /* CONTIKIMAC_CONF_COMPOWER */

  contikimac_is_on = contikimac_was_on;
  we_are_sending = 0;

  /* Determine the return value that we will return from the
     function. We must pass this value to the phase module before we
     return from the function.  */
  if(collisions > 0) {
    ret = MAC_TX_COLLISION;
  } else if(!is_broadcast && !got_strobe_ack) {
    ret = MAC_TX_NOACK;
#if TEST_NOACK  
    mac_noack_count++;
#endif
  //  printf("MAC_TX_NOACK\n");
  } else {
    ret = MAC_TX_OK;
 //   printf("MAC_TX_OK\n");
  }

#if WITH_PHASE_OPTIMIZATION
  if(is_known_receiver && got_strobe_ack) {
    PRINTF("no miss %d wake-ups %d\n",
	   packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
           strobes);
  }

  if(!is_broadcast) {
    if(collisions == 0 && is_receiver_awake == 0) {
      phase_update(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
		   encounter_time, ret);
    }
  }
#endif /* WITH_PHASE_OPTIMIZATION */

#if TEST_NOACK
  printf("mac_noack_count:%u ,mac_ack_count:%u,mac_collsion_count:%u\n",
      mac_noack_count,mac_ack_count,mac_collsion_count);
#endif

  return ret;
}
/*---------------------------------------------------------------------------*/
static void
qsend_packet(mac_callback_t sent, void *ptr)
{
  int ret = send_packet(sent, ptr, NULL, 0);
  if(ret != MAC_TX_DEFERRED) {
#if WAKEUP_NODE  
    if(get_wake_send()){
      wake_sent_callback(ret,1);
      mac_call_sent_callback(sent, ptr, MAC_TX_OK, 1);
    }else{
#endif
       mac_call_sent_callback(sent, ptr, ret, 1);
#if WAKEUP_NODE   
    }
#endif
  }
}
/*---------------------------------------------------------------------------*/
static void
qsend_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  struct rdc_buf_list *curr;
  struct rdc_buf_list *next;
  int ret;
  int is_receiver_awake;
  int pending;
  
  if(buf_list == NULL) {
    return;
  }
  /* Do not send during reception of a burst */
  if(we_are_receiving_burst) {
    /* Prepare the packetbuf for callback */
    queuebuf_to_packetbuf(buf_list->buf);
    /* Return COLLISION so the MAC may try again later */
    mac_call_sent_callback(sent, ptr, MAC_TX_COLLISION, 1);
    return;
  }
  
  /* Create and secure frames in advance */
  curr = buf_list;
  do {
    next = list_item_next(curr);
    queuebuf_to_packetbuf(curr->buf);
    if(!packetbuf_attr(PACKETBUF_ATTR_IS_CREATED_AND_SECURED)) {
      /* create and secure this frame */
      if(next != NULL) {
#if ORPL_BITMAP_SUPPORT_CONTIKIMAC_BURST   
        packetbuf_set_attr(PACKETBUF_ATTR_PENDING, 1);
#else
        packetbuf_set_attr(PACKETBUF_ATTR_PENDING, 0); 
#endif /* ORPL_BITMAP_SUPPORT_CONTIKIMAC_BURST */
      }
      packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
      if(NETSTACK_FRAMER.create() < 0) {
        PRINTF("contikimac: framer failed\n");
        mac_call_sent_callback(sent, ptr, MAC_TX_ERR_FATAL, 1);
        return;
      }
      
      packetbuf_set_attr(PACKETBUF_ATTR_IS_CREATED_AND_SECURED, 1);
      queuebuf_update_from_packetbuf(curr->buf);
    }
    curr = next;
  } while(next != NULL);
  
  /* The receiver needs to be awoken before we send */
  is_receiver_awake = 0;
  curr = buf_list;
  do { /* A loop sending a burst of packets from buf_list */
#if ORPL_BITMAP_SUPPORT_CONTIKIMAC_BURST    
    next = list_item_next(curr);
#else
    next = NULL;
#endif /* ORPL_BITMAP_SUPPORT_CONTIKIMAC_BURST */    

    /* Prepare the packetbuf */
    queuebuf_to_packetbuf(curr->buf);

    pending = packetbuf_attr(PACKETBUF_ATTR_PENDING);

    /* Send the current packet */
    ret = send_packet(sent, ptr, curr, is_receiver_awake);
    if(ret != MAC_TX_DEFERRED) {
#if WAKEUP_NODE  
    if(get_wake_send()){
       wake_sent_callback(ret,1);
       mac_call_sent_callback(sent, ptr, MAC_TX_OK, 1);
    }else{
#endif
       mac_call_sent_callback(sent, ptr, ret, 1);
#if WAKEUP_NODE   
    }
#endif
    }

    if(ret == MAC_TX_OK) {
      if(next != NULL) {
        /* We're in a burst, no need to wake the receiver up again */
        is_receiver_awake = 1;
        curr = next;
      }
    } else {
      /* The transmission failed, we stop the burst */
      next = NULL;
    }
  } while((next != NULL) && pending);
}
/*---------------------------------------------------------------------------*/
/* Timer callback triggered when receiving a burst, after having
   waited for a next packet for a too long time. Turns the radio off
   and leaves burst reception mode */
static void
recv_burst_off(void *ptr)
{
  off();
  we_are_receiving_burst = 0;
}
/*---------------------------------------------------------------------------*/
static void
input_packet(void)
{ 
  static struct ctimer ct;
  int duplicate = 0;

#if CONTIKIMAC_SEND_SW_ACK
  int original_datalen;
  uint8_t *original_dataptr;

  original_datalen = packetbuf_datalen();
  original_dataptr = packetbuf_dataptr();
#endif
  
#if TEST_FCF_PROGRAM_DELAY
  fcf_input_t1 = RTIMER_NOW();
#endif

  if(!we_are_receiving_burst) {
    off();
  }

  if(packetbuf_datalen() <= ACK_LEN) {
    /* Ignore ack packets */
    PRINTF("ContikiMAC: ignored ack\n");
    return;
  }
  /*  printf("cycle_start 0x%02x 0x%02x\n", cycle_start, cycle_start % CYCLE_TIME);*/

  if(packetbuf_totlen() > 0 && NETSTACK_FRAMER.parse() >= 0) {
#if (!UIP_CONF_IPV6_ORPL_BITMAP) 
    if(packetbuf_datalen() > 0 &&
       (linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     &linkaddr_node_addr)||
        packetbuf_holds_broadcast())) {
#endif
      /* This is a regular packet that is destined to us or to the
         broadcast address. */

      /* If FRAME_PENDING is set, we are receiving a packets in a burst */
      we_are_receiving_burst = packetbuf_attr(PACKETBUF_ATTR_PENDING);
      if(we_are_receiving_burst) {
        on();
        /* Set a timer to turn the radio off in case we do not receive
	   a next packet */
        ctimer_set(&ct, INTER_PACKET_DEADLINE, recv_burst_off, NULL);
      } else {
        off();
        ctimer_stop(&ct);
      }

#if RDC_WITH_DUPLICATE_DETECTION & (!ORPL_BITMAP_DUPLICATE_DETECTION)
      /* Check for duplicate packet. */
      duplicate = mac_sequence_is_duplicate();
      if(duplicate) {
        /* Drop the packet. */
        PRINTF("contikimac: Drop duplicate\n");
      } else {
        mac_sequence_register_seqno();
      }
#endif /* RDC_WITH_DUPLICATE_DETECTION */

#if CONTIKIMAC_CONF_COMPOWER
      /* Accumulate the power consumption for the packet reception. */
      compower_accumulate(&current_packet);
      /* Convert the accumulated power consumption for the received
         packet to packet attributes so that the higher levels can
         keep track of the amount of energy spent on receiving the
         packet. */
      compower_attrconv(&current_packet);

      /* Clear the accumulated power consumption so that it is ready
         for the next packet. */
      compower_clear(&current_packet);
#endif /* CONTIKIMAC_CONF_COMPOWER */

      PRINTDEBUG("contikimac: data (%u)\n", packetbuf_datalen());

#if CONTIKIMAC_SEND_SW_ACK
      {
        frame802154_t info154;
        frame802154_parse(original_dataptr, original_datalen, &info154);
        if(info154.fcf.frame_type == FRAME802154_DATAFRAME &&
            info154.fcf.ack_required != 0 &&
            (linkaddr_cmp((linkaddr_t *)&info154.dest_addr,
                &linkaddr_node_addr)||
            orpl_bitmap_satisfy_fwd((linkaddr_t *)&info154.dest_addr))) {
          uint8_t ackdata[ACK_LEN] = {0, 0, 0};

          we_are_sending = 1;
          ackdata[0] = FRAME802154_ACKFRAME;
          ackdata[1] = 0;
          ackdata[2] = info154.seq;
          
          NETSTACK_RADIO.send(ackdata, ACK_LEN);
          we_are_sending = 0;
        }
      }
#endif /* CONTIKIMAC_SEND_SW_ACK */

      if(!duplicate) {
        NETSTACK_MAC.input();
      }
      return;
#if (!UIP_CONF_IPV6_ORPL_BITMAP)       
    } else {
      PRINTDEBUG("contikimac: data not for us\n");
    }
#endif
  } else {
    PRINTF("contikimac: failed to parse (%u)\n", packetbuf_totlen());
  }
}
/*--------------------------------------------------------------------------*/
static void 
set_rdc_channel_check_interval(){
  rdc_channel_check_interval = rdc_inactive_channel_check_interval * rdc_active_channel_check_rate;
}
/*--------------------------------------------------------------------------*/
//static 
// void 
// set_cycletime(){
//   CYCLE_TIME = RTIMER_ARCH_SECOND / rdc_active_channel_check_rate;
  
//   //printf("CYCLE_TIME :%lu\n",CYCLE_TIME);
//   STROBE_TIME = (CYCLE_TIME + 2 * CHECK_TIME);
// //  rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1, powercycle_wrapper, NULL);
// }
//zhangwei set changed for load balance
void 
set_cycletime(uint16_t cycle_time)
{
   // printf("CYCLE_TIME2 :%lu\n",CYCLE_TIME);
    #if ROOTNODE
      STROBE_TIME =( RTIMER_ARCH_SECOND / rdc_active_channel_check_rate + 2 * CHECK_TIME);
    #else
      uint16_t cycle_time1 = RTIMER_ARCH_SECOND / rdc_active_channel_check_rate;

      CYCLE_TIME = MAX(cycle_time1,cycle_time);

      STROBE_TIME = (CYCLE_TIME + 2 * CHECK_TIME);
    #endif
  //  rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1, powercycle_wrapper, NULL);
}
uint16_t get_cycle_time(void){
  return CYCLE_TIME;
}
/*--------------------------------------------------------------------------*/
void 
set_rdc_active_channel_check_rate(uint8_t channel_check_rate){
  if(channel_check_rate!= 0 && channel_check_rate != 0xFF){
    rdc_active_channel_check_rate = channel_check_rate;
      // set_cycletime();
//zhangwei set changed for load balance
  set_cycletime(RTIMER_ARCH_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE);

    set_rdc_channel_check_interval();
  }
}
/*--------------------------------------------------------------------------*/
void
set_rdc_inactive_channel_check_interval(uint8_t channel_check_interval){
  if(channel_check_interval!= 0 && channel_check_interval != 0xFF){
    rdc_inactive_channel_check_interval = channel_check_interval;
    set_rdc_channel_check_interval();
  }
}
/*--------------------------------------------------------------------------*/
uint8_t 
get_rdc_active_channel_check_rate(){
   return rdc_active_channel_check_rate ;
}
uint8_t
get_rdc_inactive_channel_check_interval(){
   return rdc_inactive_channel_check_interval;
}
/*--------------------------------------------------------------------------*/
void
rdc_turn_radio_off(void)
{
  if(radio_is_on == 0 && contikimac_keep_radio_on == 0) {
    NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  radio_is_on = 0;
  PT_INIT(&pt);

  // set_cycletime();
//zhangwei set changed for load balance
  set_cycletime(RTIMER_ARCH_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE);

  rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1, powercycle_wrapper, NULL);

  contikimac_is_on = 1;

#if WITH_PHASE_OPTIMIZATION
  phase_init();
#endif /* WITH_PHASE_OPTIMIZATION */

}
/*---------------------------------------------------------------------------*/
static int
turn_on(void)
{
  if(contikimac_is_on == 0) {
    contikimac_is_on = 1;
    contikimac_keep_radio_on = 0;
    rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1, powercycle_wrapper, NULL);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on)
{
  contikimac_is_on = 0;
  contikimac_keep_radio_on = keep_radio_on;
  if(keep_radio_on) {
    radio_is_on = 1;
    return NETSTACK_RADIO.on();
  } else {
    radio_is_on = 0;
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
duty_cycle(void)
{
  //zhangwei set changed for load balance
  uint32_t  cycle_time= (RTIMER_ARCH_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE);

  return (1ul * CLOCK_SECOND * cycle_time) / RTIMER_ARCH_SECOND;

  // return (1ul * CLOCK_SECOND * CYCLE_TIME) / RTIMER_ARCH_SECOND;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver contikimac_fcf_driver = {
  "ContikiMAC-fcf",
  init,
  qsend_packet,
  qsend_list,
  input_packet,
  turn_on,
  turn_off,
  duty_cycle,
};
/*---------------------------------------------------------------------------*/
uint16_t
contikimac_debug_print(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
#endif


