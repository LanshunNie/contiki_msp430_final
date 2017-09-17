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
 * \file
 *	Public API declarations for ORPL bitmap.
 * \author
 *	@yang
 *
 */

#ifndef ORPL_BITMAP_H
#define ORPL_BITMAP_H

#include "orpl-bitmap-conf.h"
#include "net/ip/uip.h"
#include "net/linkaddr.h"
#include "net/rpl/rpl.h"
#include "net/mac/frame802154.h"

/* RPL bitmap message types */
#define ORPL_BITMAP_CODE                0x00  
#define ORPL_BITMAP_CODE_ACK            0x01  

enum bitmap_direction_e {
  direction_none,
  direction_up,
  direction_down,
};

struct bitmap_info {
  enum bitmap_direction_e direction;
  uint16_t node_rank;
#if ORPL_BITMAP_DUPLICATE_DETECTION
  uint16_t seqno;
#endif
};

extern struct bitmap_info  bitmap_send_info;
extern struct bitmap_info  bitmap_receive_info;

#if ORPL_BITMAP_DUPLICATE_DETECTION
/**
 * \brief      Tell whether the packetbuf is a duplicate packet
 * \return     Non-zero if the packetbuf is a duplicate packet, zero otherwise
 *
 *             This function is used to check for duplicate packet by comparing
 *             the sequence number of the incoming packet with the last few ones
 *             we saw.
 */
uint8_t orpl_bitmap_duplicate_detect(uip_ipaddr_t * src_ipaddr,uip_ipaddr_t * dest_ipaddr);

/**
 * \brief      Register the sequence number of the packetbuf
 *
 *             This function is used to add the sequence number of the incoming
 *             packet to the history.
 */
void orpl_bitmap_duplicate_register(uip_ipaddr_t * src_ipaddr,uip_ipaddr_t * dest_ipaddr);

uint16_t orpl_bitmap_get_current_seqno();

inline uint16_t get_orpl_packet_seqno();

#endif /* ORPL_BITMAP_DUPLICATE_DETECTION */

void orpl_schedule_bitmap();
/* RPL bitmap initialization */
void orpl_bitmap_init();
/* Build a global link-layer address from an IPv6 based on its UUID64 */
void lladdr_from_ipaddr_uuid(uip_lladdr_t *lladdr, const uip_ipaddr_t *ipaddr);
/* Function called when some  trickle timer expires */
void orpl_bitmap_trickle_callback();
/* rpl bitmap networking limit func */
uint8_t orpl_bitmap_limit_networking(const linkaddr_t *lladdr);
/* Returns current rank of the node */
rpl_rank_t orpl_bitmap_current_rank();

rpl_rank_t orpl_bitmap_root_rank();

/* Routing downwards conditions*/
uint8_t orpl_bitmap_satisfy_fwd(const frame802154_t *info154);

uint8_t orpl_bitmap_panid_limit(uint16_t dest_pid);

uip_ipaddr_t *orpl_get_parent_ipaddr(void);

linkaddr_t *orpl_get_parent_linkaddr(void);
#if 0
/* Returns a node-id from a node's link-layer address */
uint16_t node_id_from_linkaddr(const linkaddr_t *addr);
/* Returns a node-id from a node's IPv6 address */
uint16_t node_id_from_ipaddr(const uip_ipaddr_t *addr);
/* Sets an IPv6 from a node-id */
void set_ipaddr_from_id(uip_ipaddr_t *ipaddr, uint16_t id);
/* Sets an linkaddr from a node-id*/
void set_linkaddr_from_id(linkaddr_t *lladdr, uint16_t id);
#endif

#endif /*ORPL_BITMAP_H*/
