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
 *         ICMP6 I/O for ORPL bitmap control messages.
 *  author @yang
 */
/**
 * \addtogroup uip6
 * @{
 */

#include "net/ip/tcpip.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-nd6.h"
#include "net/ipv6/uip-icmp6.h"
#include "net/rpl/rpl-private.h"
#include "net/packetbuf.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "net/orpl-bitmap/orpl-bitmap.h"
#include "net/orpl-bitmap/orpl-routing-set.h"
#include "net/orpl-bitmap/orpl-bitmap-icmp6.h"

#include "contiki-conf.h"
#if UIP_CONF_MULTI_SUBNET
#include "multi-subnet.h"
#endif

#include <limits.h>
#include <string.h>

#if ORPL_MOP_DOWNWARD_STORING
  
#define DEBUG DEBUG_NONE

#include "net/ip/uip-debug.h"

#define UIP_IP_BUF       ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_ICMP_BUF     ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])
#define UIP_ICMP_PAYLOAD ((unsigned char *)&uip_buf[uip_l2_l3_icmp_hdr_len])
/*---------------------------------------------------------------------------*/
static void orpl_bitmap_input(void);
#if DEBUG
static void orpl_bitmap_ack_input(void);
static uint8_t orpl_bitmap_sequence = RPL_LOLLIPOP_INIT;
#endif
/*---------------------------------------------------------------------------*/
/* Initialise RPL bitmap ICMPv6 message handlers */
UIP_ICMP6_HANDLER(orpl_bitmap_handler, ICMP6_ORPL_BITMAP,ORPL_BITMAP_CODE,orpl_bitmap_input);
#if DEBUG
UIP_ICMP6_HANDLER(orpl_bitmap_ack_handler, ICMP6_ORPL_BITMAP,ORPL_BITMAP_CODE_ACK,orpl_bitmap_ack_input);
#endif
/*---------------------------------------------------------------------------*/
static void 
orpl_bitmap_input(void)
{
  uip_ipaddr_t bitmap_sender_addr;
  uint16_t sender_rank;
  unsigned char *buffer;
#if DEBUG 
  uint8_t sequence;
#endif  
  uint16_t bit_count_before;
  uint16_t bit_count_after;
  struct routing_set_s rs;
  int pos;
  uip_ds6_nbr_t *nbr;
  rpl_rank_t curr_rank;

#if SUBNET_PANID_CONF_LIMIT 
    /* drop not our panid netsync message */
    if(get_own_subnet_panid() != get_link_receive_subnet_panid()) {
      goto discard;
    }
#endif

  uip_ipaddr_copy(&bitmap_sender_addr, &UIP_IP_BUF->srcipaddr);
  PRINTF("RPL: Received a bitmap from ");
  PRINT6ADDR(&bitmap_sender_addr);
  PRINTF("\n");
  
  if((nbr = uip_ds6_nbr_lookup(&bitmap_sender_addr)) == NULL) {
    if((nbr = uip_ds6_nbr_add(&bitmap_sender_addr,
                           (uip_lladdr_t *)packetbuf_addr(PACKETBUF_ADDR_SENDER),
                            0, NBR_REACHABLE)) != NULL) {
      /* set reachable timer */
      stimer_set(&nbr->reachable, UIP_ND6_REACHABLE_TIME / 1000);
      PRINTF("RPL: Neighbor added to neighbor cache ");
      PRINT6ADDR(&bitmap_sender_addr);
      PRINTF(", ");
      PRINTLLADDR((uip_lladdr_t *)packetbuf_addr(PACKETBUF_ADDR_SENDER));
      PRINTF("\n");
    } 
  } else {
    /* set reachable timer */
    stimer_set(&nbr->reachable, UIP_ND6_REACHABLE_TIME / 1000);
    PRINTF("RPL: Neighbor already in neighbor cache\n");
  }

  buffer = UIP_ICMP_PAYLOAD;
  pos = 0;
 
#if DEBUG 
  sequence = buffer[pos++];
#endif   

  curr_rank = orpl_bitmap_current_rank();
  memcpy(&sender_rank,buffer + pos,2);
 // printf("sender addr:%02x,sender rank:%u\n",bitmap_sender_addr.u8[15],sender_rank);
 // printf("curr_rank:%u\n",curr_rank);
  pos += 2;
  if(sender_rank > ORPL_RANK_W && curr_rank > sender_rank - ORPL_RANK_W){
     goto discard;
  }
  bit_count_before = orpl_routing_set_count_bits();
  orpl_routing_set_insert(&bitmap_sender_addr);
  memcpy(&rs,buffer + pos,sizeof(struct routing_set_s));
  /* The node is a child, merge its routing set in ours */
  orpl_routing_set_merge(&rs);     
  /* unicast our routing set again if it has changed */
  bit_count_after = orpl_routing_set_count_bits();
  if(bit_count_after != bit_count_before&&
     curr_rank != orpl_bitmap_root_rank()) {
   //  orpl_bitmap_output();
     orpl_schedule_bitmap();
  }

#if DEBUG  
   orpl_bitmap_ack_output(&bitmap_sender_addr, sequence);
#endif

 discard:
  uip_clear_buf();  
}
/*---------------------------------------------------------------------------*/
/* multicast our routing set to our potential parent*/
void
orpl_bitmap_output(){
  unsigned char *buffer;
  int pos;
  struct routing_set_s*rs;
  uip_ipaddr_t addr;
  rpl_rank_t curr_rank;

  rs = orpl_routing_set_get_active();
  uip_create_linklocal_rplnodes_mcast(&addr);
  buffer = UIP_ICMP_PAYLOAD;
  pos = 0;
  curr_rank = orpl_bitmap_current_rank();

#if DEBUG
  RPL_LOLLIPOP_INCREMENT(orpl_bitmap_sequence);
  buffer[pos++] = orpl_bitmap_sequence;
#endif

  memcpy(buffer + pos,&curr_rank, sizeof(rpl_rank_t));
  pos += 2;
  memcpy(buffer + pos,rs, sizeof(struct routing_set_s));
  pos += sizeof(struct routing_set_s);

  PRINTF("RPL: Sending RPL bitmap to");
  PRINT6ADDR(&addr);
  PRINTF("\n");
  
  uip_icmp6_send(&addr,ICMP6_ORPL_BITMAP, ORPL_BITMAP_CODE, pos);
}
/*---------------------------------------------------------------------------*/
#if DEBUG
static void
orpl_bitmap_ack_input(void)
{
  unsigned char *buffer;
  uint8_t sequence;

  buffer = UIP_ICMP_PAYLOAD;
  sequence = buffer[0];

  PRINTF("RPL bitmap: Received a ACK with sequence number %u",sequence);
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("\n");
  uip_clear_buf();
}
#endif /* DEBUG */
/*---------------------------------------------------------------------------*/
void
orpl_bitmap_ack_output(uip_ipaddr_t *dest, uint8_t sequence)
{
#if DEBUG  
  unsigned char *buffer;

  PRINTF("RPL: Sending a DAO ACK with sequence number %d to ", sequence);
  PRINT6ADDR(dest);
  PRINTF("\n");

  buffer = UIP_ICMP_PAYLOAD;
  buffer[0] = sequence;
  uip_icmp6_send(dest, ICMP6_ORPL_BITMAP, ORPL_BITMAP_CODE_ACK, 1);
#endif
}
/*---------------------------------------------------------------------------*/
void
orpl_bitmap_icmp6_register_handlers()
{
  uip_icmp6_register_input_handler(&orpl_bitmap_handler);
#if DEBUG   
  uip_icmp6_register_input_handler(&orpl_bitmap_ack_handler);
#endif
}
/*---------------------------------------------------------------------------*/
#endif


