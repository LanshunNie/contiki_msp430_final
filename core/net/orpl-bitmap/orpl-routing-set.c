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
 *         Routing set support for ORPL.  We implement routing sets
 *         as Bloom filters, and have a generic driver interface for
 *         hashing (get_hash). Routing sets can be turned into simple
 *         bitmaps when using a collision-free hash that maps every
 *         global IPv6 in the network to a unique position in the set.
 *
 * \author Simon Duquennoy <simonduq@sics.se>
 */

#include "net/ip/uip.h"
#include "orpl-bitmap.h"
#include "orpl-routing-set.h"
#include "node-id.h"
#include <string.h>
#include <stdio.h>

#if ORPL_MOP_DOWNWARD_STORING
/* We maintain two routing sets, one "active" and one "warmup" to implement ageing. */
static struct routing_set_s routing_sets[2];
/* Index of the currently active set. 1-current is the warmup one. */
volatile static uint8_t active_index;

static uint8_t rs_get_bit(struct routing_set_s *rs, uint16_t i);
 
/* In the bitmap case, we uniquely map all global ipv6 in the
   * network to a unique index, based on a deployment-specific
   * node_id_from_ipaddr(ipv6) function */
/* Returns the 16-bit hash of a given global IPv6 */

static uint16_t
get_hash_ipv6(const uip_ipaddr_t *addr)
{
  return ((addr->u8[14]&(0xff>>(16-NODE_BIT_NUM)))<<8)+addr->u8[15];
}

static uint16_t
get_hash_lladdr(const linkaddr_t *addr)
{
  return ((addr->u8[6]&(0xff>>(16-NODE_BIT_NUM)))<<8)+addr->u8[7];
}

/* Get a bit in a routing set */
static uint8_t
rs_get_bit(struct routing_set_s *rs, uint16_t i) {
  return (rs->u8[i/8] & (1 << (i%8))) != 0;
}

/* Initializes the global double routing set */
void
orpl_routing_set_init()
{
  memset(routing_sets, 0, sizeof(routing_sets));
 // printf("routing_sets length:%u\n",sizeof(routing_sets));
  active_index = 0;
}

/* Returns a pointer to the currently active routing set */
struct routing_set_s *
orpl_routing_set_get_active() {
  return &routing_sets[active_index];
}

/* Inserts a global IPv6 in the global double routing set */
void
orpl_routing_set_insert(const uip_ipaddr_t *ipv6)
{
  uint16_t hash = get_hash_ipv6(ipv6);
  /* For each hash, set a bit in both routing sets */
 // printf("ipv6 addr: %02x%02x,hash:%u\n",ipv6->u8[14],ipv6->u8[15],hash);
 // printf("ROUTING_SET_M:%u ,mod:%u \n",ROUTING_SET_M,hash);

  /* Set a bit in a routing set */
 // routing_sets[0].u8[hash/8] = routing_sets[0].u8[hash/8]|(1 << (hash%8));
 // routing_sets[1].u8[hash/8] = routing_sets[1].u8[hash/8]|(1 << (hash%8));
  
    routing_sets[0].u8[hash/8] |= (1 << (hash%8));
    routing_sets[1].u8[hash/8] |= (1 << (hash%8));
  
 // printf("hash: %d,hash/8:%d,hash mod8: %d\n",hash,hash/8,hash%8);
 // printf("insert routing_set 0:%02x\n",routing_sets[0].u8[0]);
 // printf("insert routing_set 1:%02x\n",routing_sets[1].u8[0]);  
} 

/* Merges a routing set into our global double routing set */
void
orpl_routing_set_merge(const struct routing_set_s *rs)
{
  int i;
 // printf("merge u8[0]:%02x\n",rs->u8[0]);
  
  for(i=0; i<sizeof(struct routing_set_s); i++) {
    /* We merge into both active and warmup routing sets.
     * Merging is ORing */
   // routing_sets[0].u8[i] = routing_sets[0].u8[i]|rs->u8[i];
   // routing_sets[1].u8[i] = routing_sets[1].u8[i]|rs->u8[i];

    routing_sets[0].u8[i] |= rs->u8[i];
    routing_sets[1].u8[i] |= rs->u8[i];
  } 
 // printf("merge routing_set 0:%02x\n",routing_sets[0].u8[0]);
}

/* Checks if our global double routing set contains an given IPv6 */
int
orpl_routing_set_contains_ipv6(const uip_ipaddr_t *ipv6)
{
  int contains = 1;
  uint16_t hash = get_hash_ipv6(ipv6);
  /* For each hash, check a bit in the bitmap */
  /* Check against the active routing set */
  if(rs_get_bit(orpl_routing_set_get_active(), hash) == 0) {
    /* If one bucket is empty, then the element isn't included in the bitmap */
     contains = 0;
  } 
  return contains;
}
/* Checks if our global double routing set contains an given lladdr */
int 
orpl_routing_set_contains_lladdr(const linkaddr_t *lladdr)
{
  int contains = 1;
  uint16_t hash = get_hash_lladdr(lladdr);
  /* For each hash, check a bit in the bitmap */
  /* Check against the active routing set */
  if(rs_get_bit(orpl_routing_set_get_active(), hash) == 0) {
    /* If one bucket is empty, then the element isn't included in the bitmap */
     contains = 0;
  }  
  return contains;
}

/* Swap active and warmup routing sets for ageing */
void
orpl_routing_set_swap()
{
  /* Swap active flag */
  active_index = 1 - active_index;
  /* Reset the newly inactive routing set */
  memset(routing_sets[1 - active_index].u8, 0, sizeof(struct routing_set_s));
}

/* Returns the number of bits set in the active routing set */
int
orpl_routing_set_count_bits()
{
  int i;
    int cnt = 0;
    for(i=0; i<ROUTING_SET_M; i++) {
      if(rs_get_bit(orpl_routing_set_get_active(), i)) {
        cnt++;
      }
    }
  return cnt;
}

#endif

