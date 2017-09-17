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

#include "contiki-conf.h"
#include "sys/node-id.h"
#include "net/rpl/rpl.h"
#include "random.h"
#include "orpl-bitmap.h"
#include "orpl-bitmap-icmp6.h" 
#include "orpl-routing-set.h"
 
#include "net/mac/frame802154.h"

#if UDP_CONF_FRAG
#include "udp-fragment.h"
#endif
 
#if UIP_CONF_MULTI_SUBNET
#include "multi-subnet.h"
#endif

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"
#include <string.h>

#if ORPL_MOP_DOWNWARD_STORING
static struct ctimer orpl_bitmap_timer;
#endif
struct bitmap_info  bitmap_send_info;
struct bitmap_info  bitmap_receive_info;

#if ORPL_BITMAP_DUPLICATE_DETECTION
/* Seqno of the next packet to be sent */
static uint16_t current_seqno = 0;
static uint8_t  init_seqno_flag = 0;

struct orpl_bitmap_seqno {
   uint16_t seqno;
   uint16_t dest_addr;
   uint16_t src_addr;
};

#define ORPL_BITMAP_MAX_SEQNONUM   4  //8
static struct orpl_bitmap_seqno received_orpl_bitmap_seqnos[ORPL_BITMAP_MAX_SEQNONUM];
#endif /* ORPL_BITMAP_DUPLICATE_DETECTION */
/*---------------------------------------------------------------------------*/
#if ORPL_BITMAP_DUPLICATE_DETECTION
static void
orpl_init_current_seqno(){
  current_seqno=random_rand();
}
#endif /* ORPL_BITMAP_DUPLICATE_DETECTION */
/*---------------------------------------------------------------------------*/
#if ORPL_MOP_DOWNWARD_STORING
static void
handle_orpl_bitmap_timer(void *ptr)
{
  orpl_bitmap_output();
  ctimer_stop(&orpl_bitmap_timer);
}
/*---------------------------------------------------------------------------*/
static void
schedule_bitmap(clock_time_t latency)
{
  clock_time_t expiration_time;

  if(latency != 0) {
    expiration_time = latency / 2 +
      (random_rand() % (latency));
  } else {
    expiration_time = 0;
  }
  PRINTF("ORPL: Scheduling bitmap timer %u ticks in the future\n",
           (unsigned)expiration_time);
  ctimer_set(&orpl_bitmap_timer, expiration_time,
             handle_orpl_bitmap_timer,NULL);
}
/*---------------------------------------------------------------------------*/
/* Function called when the some DIO trickle timer expires */
void
orpl_bitmap_trickle_callback()
{
  /* Swap routing sets to implement ageing */
  PRINTF("ORPL bitmap: swapping routing sets\n");
  orpl_routing_set_swap();
}
/*---------------------------------------------------------------------------*/
void
orpl_schedule_bitmap()
{
  schedule_bitmap(ORPL_BITMAP_DELAY);
}
#endif
/*---------------------------------------------------------------------------*/
/* orpl bitmap networking limit func */
uint8_t orpl_bitmap_limit_networking(const linkaddr_t *lladdr)
{
  if((memcmp(lladdr,&linkaddr_node_addr,6) == 0)&&
    (lladdr->u8[6]>>(NODE_BIT_NUM-8))==(linkaddr_node_addr.u8[6]>>(NODE_BIT_NUM-8))){
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Returns current rank of the node */
rpl_rank_t
orpl_bitmap_current_rank()
{
  rpl_dag_t *dag = rpl_get_any_dag();
  return dag == NULL ? 0xffff : dag->rank;
}

rpl_rank_t
orpl_bitmap_root_rank()
{
  rpl_dag_t *dag = rpl_get_any_dag();
  return dag == NULL ? 0xffff : dag->instance->min_hoprankinc;
}
/*---------------------------------------------------------------------------*/
uint8_t
orpl_bitmap_satisfy_fwd(const frame802154_t *info154)
{
#if !UIP_CONF_ROUTER   
    return 0;
#endif

#if SUBNET_PANID_CONF_LIMIT 
  if(!subnet_panid_limit(info154->dest_pid)) {
    return 0;
  }
#endif

  //if(linkaddr_cmp((linkaddr_t *)&(info154->dest_addr),&linkaddr_node_addr)) {
  //  return 1;
 // }

  rpl_rank_t curr_rank = orpl_bitmap_current_rank();
  if(bitmap_receive_info.direction == direction_up) {
    /* Routing upwards. ACK if our rank is better. */
    if(bitmap_receive_info.node_rank > ORPL_RANK_W && curr_rank <= bitmap_receive_info.node_rank - ORPL_RANK_W) {
 //     printf("fwd up\n");
      return 1;
    }
#if ORPL_MOP_DOWNWARD_STORING
     else {
    /* We don't route upwards, now check if we are a common ancester of the source
     * and destination. We do this by checking our routing set against the destination. */
      if(orpl_routing_set_contains_lladdr((linkaddr_t *)&(info154->dest_addr))) {
      /* Traffic is going up but we have destination in our routing set.
        * Ack it and start routing downwards (towards the destination) */
        return 1;
      } 
    } 
#endif    
  } 
#if ORPL_MOP_DOWNWARD_STORING
  else if(bitmap_receive_info.direction == direction_down){
  /* Routing downwards. return 1 if destination is in subdodag and we have a worse rank */
    if((curr_rank > ORPL_RANK_W && curr_rank - ORPL_RANK_W >= bitmap_receive_info.node_rank
        && orpl_routing_set_contains_lladdr((linkaddr_t *)&(info154->dest_addr)))) {
      return 1;
    }
  }
#endif
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Build a global link-layer address from an IPv6 based on its UUID64 */
void
lladdr_from_ipaddr_uuid(uip_lladdr_t *lladdr, const uip_ipaddr_t *ipaddr)
{
#if (UIP_LLADDR_LEN == 8)
  memcpy(lladdr, ipaddr->u8 + 8, UIP_LLADDR_LEN);
  lladdr->addr[0] ^= 0x02;
#endif
}
/*---------------------------------------------------------------------------*/
#if ORPL_BITMAP_DUPLICATE_DETECTION
uint8_t 
orpl_bitmap_duplicate_detect(uip_ipaddr_t * src_ipaddr,uip_ipaddr_t * dest_ipaddr)
{
  int i;
 // printf("orpl bitmap receive seqno: %u\n",bitmap_receive_info.seqno);
  uint16_t src_addr = ((src_ipaddr->u8[14])<<8) + (src_ipaddr->u8[15]);
  uint16_t dest_addr = ((dest_ipaddr->u8[14])<<8) + (dest_ipaddr->u8[15]);
  for(i = 0; i < ORPL_BITMAP_MAX_SEQNONUM; i++) {    
    if(received_orpl_bitmap_seqnos[i].seqno == bitmap_receive_info.seqno&&
       received_orpl_bitmap_seqnos[i].src_addr == src_addr&&
       received_orpl_bitmap_seqnos[i].dest_addr == dest_addr){ 
       return 1;  
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void 
orpl_bitmap_duplicate_register(uip_ipaddr_t * src_ipaddr,uip_ipaddr_t * dest_ipaddr)
{
  uint8_t i; 
  for(i = ORPL_BITMAP_MAX_SEQNONUM - 1; i > 0; --i) {
     received_orpl_bitmap_seqnos[i] = received_orpl_bitmap_seqnos[i - 1];
  }
  received_orpl_bitmap_seqnos[0].seqno = bitmap_receive_info.seqno;
  received_orpl_bitmap_seqnos[0].src_addr = ((src_ipaddr->u8[14])<<8) + (src_ipaddr->u8[15]);
  received_orpl_bitmap_seqnos[0].dest_addr = ((dest_ipaddr->u8[14])<<8) + (dest_ipaddr->u8[15]);
}
/*---------------------------------------------------------------------------*/
uint16_t
orpl_bitmap_get_current_seqno()
{
  uint16_t ret;
  if(!init_seqno_flag){
    init_seqno_flag =1;
    orpl_init_current_seqno();
  }
  ret = current_seqno;
  current_seqno ++;
  if(current_seqno == 0){
    current_seqno++;
  }   
  return ret;
}
/*---------------------------------------------------------------------------*/
inline uint16_t
get_orpl_packet_seqno(){
  uint16_t orpl_packet_seqno;

  memcpy(&orpl_packet_seqno,&uip_buf[UIP_LLH_LEN +uip_ext_len+UIP_IPUDPH_LEN],2);
  return orpl_packet_seqno;
}
#endif /* ORPL_BITMAP_DUPLICATE_DETECTION */
/*---------------------------------------------------------------------------*/
/* RPL bitmap initialization */
void
orpl_bitmap_init()
{
#if UDP_CONF_FRAG
    udp_fragment_init();
#endif
#if ORPL_MOP_DOWNWARD_STORING  
  orpl_routing_set_init();
  orpl_bitmap_icmp6_register_handlers();
#endif
}
/*---------------------------------------------------------------------------*/
uint8_t
orpl_bitmap_panid_limit(uint16_t dest_pid)
{
  if(dest_pid != IEEE802154_PANID && dest_pid != FRAME802154_BROADCASTPANDID){
    /* Packet to another PAN */
    PRINTF("Packet to another pan %u\n", dest_pid);
    return 0;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
// ipaddr is 0xfe80 start:

uip_ipaddr_t *
orpl_get_parent_ipaddr(void){
#if 0
  uip_ipaddr_t *parent_ipaddr =NULL;
  rpl_dag_t *dag = rpl_get_any_dag();

  if(dag == NULL || dag->instance == NULL){
    return NULL;
  }

  uip_ds6_defrt_t *def_route = dag->instance->def_route;
  if(def_route != NULL){
    parent_ipaddr = &(def_route->ipaddr);
  }
#else
  uip_ipaddr_t *parent_ipaddr =NULL;
  parent_ipaddr = uip_ds6_defrt_choose(); 
#endif

  /*
  if(dag->preferred_parent != NULL) {
    PRINTF("Preferred parent: ");
    PRINT6ADDR(rpl_get_parent_ipaddr(dag->preferred_parent));
    PRINTF("\n");
  }
  */

  return parent_ipaddr;
}
/*---------------------------------------------------------------------------*/
linkaddr_t *
orpl_get_parent_linkaddr(void){
  linkaddr_t * dest = NULL;
  uip_ipaddr_t* parent_ipaddr = orpl_get_parent_ipaddr();
  if(parent_ipaddr !=NULL){
    uip_ds6_nbr_t *nbr = uip_ds6_nbr_lookup(parent_ipaddr);
    if(nbr != NULL){
      dest = (linkaddr_t *)uip_ds6_nbr_get_ll(nbr);
    }
  }
  return dest;
}
/*---------------------------------------------------------------------------*/
#if 0

/* Returns a node-id from a node's IPv6 address */
uint16_t
node_id_from_ipaddr(const uip_ipaddr_t *addr)
{
  // printf("index:%u\n",((addr->u8[14]&(0xff>>(16-NODE_BIT_NUM)))<<8)+addr->u8[15]);
  if(addr == NULL) {
    return 0;
  } else {
    return ((addr->u8[14]&(0xff>>(16-NODE_BIT_NUM)))<<8)+addr->u8[15];
  }  
}

/* Returns a node-id from a node's linkaddr */
uint16_t
node_id_from_linkaddr(const linkaddr_t *addr)
{
  if(addr == NULL) {
    return 0;
  } else {
    return ((addr->u8[6]&(0xff>>(16-NODE_BIT_NUM)))<<8)+addr->u8[7];
  }
}

/* Sets an IPv6 from a node-id */
void
set_ipaddr_from_id(uip_ipaddr_t *ipaddr, uint16_t id)  //??
{
  linkaddr_t lladdr;
  uip_ip6addr_t *global_ipv6 = NULL;
  uip_ds6_addr_t * addr_desc = uip_ds6_get_global(ADDR_PREFERRED);
  if(addr_desc != NULL) {
     global_ipv6 = &addr_desc->ipaddr;
#if UIP_CONF_IPV6_RPL
     rpl_dag_t *dag = rpl_get_any_dag();
     if(dag) {
        uip_ipaddr_copy(&ipaddr,global_ipv6);  /* override prefix */
//        memcpy(&ipaddr, &global_ipv6, 8); /* override prefix */
     }
#endif
     set_linkaddr_from_id(&lladdr, id);
     uip_ds6_set_addr_iid(ipaddr, (uip_lladdr_t*)&lladdr); 
  }else{
    printf("no global_ipv6 addr\n");
  }
}

/* Sets a linkaddr from a node-id */
void
set_linkaddr_from_id(linkaddr_t *lladdr, uint16_t id)
{
  linkaddr_copy(lladdr,&linkaddr_node_addr);
  lladdr->u8[6] |= (id>>8&(0xff>>(16-NODE_BIT_NUM)));
  lladdr->u8[7] = id;
}

#endif


 