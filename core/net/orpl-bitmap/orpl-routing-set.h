/*
 * Copyright (c) 2013, Swedish Institute of Computer Science.
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
 */
/**
 * \file
 *         Header file for routing-set.c. We implement routing sets
 *         as Bloom filters, and have a generic driver interface for
 *         hashing (get_hash). Routing sets can be turned into simple
 *         bitmaps when using a collision-free hash that maps every
 *         global IPv6 in the network to a unique position in the set.
 *
 * \author Simon Duquennoy <simonduq@sics.se>
 */

#ifndef RPL_ROUTING_SET_H
#define RPL_ROUTING_SET_H

#include "contiki.h"
#include "orpl-bitmap-conf.h"
#if ORPL_MOP_DOWNWARD_STORING
/* A routing set is a bitmap of size ROUTING_SET_M bits */
struct routing_set_s {
  unsigned char u8[ROUTING_SET_M / 8];
};

/* Initializes the global double routing set */
void orpl_routing_set_init();
/* Returns a pointer to the currently active routing set */
struct routing_set_s *orpl_routing_set_get_active();
/* Inserts a global IPv6 in the global double routing set */
void orpl_routing_set_insert(const uip_ipaddr_t *ipv6);
/* Merges a routing set into our global double routing set */
void orpl_routing_set_merge(const struct routing_set_s *rs);
/* Checks if our global double bloom filter contains an given IPv6 */
int orpl_routing_set_contains_ipv6(const uip_ipaddr_t *ipv6);
/* Checks if our global double bloom filter contains an given lladdr */
int orpl_routing_set_contains_lladdr(const linkaddr_t *lladdr);
/* Swap active and warmup routing sets for ageing */
void orpl_routing_set_swap();
/* Returns the number of bits set in the active routing set */
int orpl_routing_set_count_bits();

#endif /* ORPL_MOP_DOWNWARD_STORING */
#endif /* RPL_ROUTING_SET_H */

