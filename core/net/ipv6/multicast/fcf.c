/*
 * Copyright (c) 2010, Loughborough University - Computer Science
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
 */

/**
 * \addtogroup fcf-multicast
 * @{
 */
/**
 * \file
 *    This file implements 'Stateless Multicast RPL Forwarding' (scf)
 *
 * \author
 *    @yang
 */

#include "contiki.h"
#include "contiki-net.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "net/ipv6/multicast/uip-mcast6-route.h"
#include "net/ipv6/multicast/uip-mcast6-stats.h"
#include "net/ipv6/multicast/fcf.h"
#include "net/rpl/rpl.h"
#include "net/netstack.h"
#include "ctimer.h"
#include <string.h>

#if UIP_CONF_IPV6_ORPL_BITMAP
#include "net/orpl-bitmap/orpl-bitmap.h"
#endif

#if LOW_LATENCY
#include "low-latency.h"
#endif

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"
/*---------------------------------------------------------------------------*/
/* Macros */
/*---------------------------------------------------------------------------*/
#define FCF_FWD_MAX_COUNT  1
#define FCF_SEQ_BUF_MAX    4

#define HBHO_OPT_TYPE            0x0C
#define HBHO_OPT_LEN             2
#define HBHO_TOTAL_LEN           8
/*****************************************************************************/
#if !ORPL_BITMAP_DUPLICATE_DETECTION
/* Multicast HBH Option */
struct hbho_mcast {
  uint8_t type;
  uint8_t len;
  uint8_t seq_id_msb;                
  uint8_t seq_id_lsb;
  uint8_t padn_type;            /* 1: PadN */
  uint8_t padn_len;             /* 0->2 bytes */
};
#endif
/*---------------------------------------------------------------------------*/
/* Internal Data */
/*---------------------------------------------------------------------------*/
static struct fcf_seq_array seq_array[FCF_SEQ_BUF_MAX];
static uint8_t  next_del_p ;
static uint16_t last_seq;

static uint16_t receive_count = 0;

#if !ORPL_BITMAP_DUPLICATE_DETECTION
static struct hbho_mcast *lochbhmptr;
#endif
/*---------------------------------------------------------------------------*/
/* uIPv6 Pointers */
/*---------------------------------------------------------------------------*/
#define UIP_IP_BUF        ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_EXT_BUF       ((struct uip_ext_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])
#define UIP_EXT_BUF_NEXT  ((uint8_t *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN + HBHO_TOTAL_LEN])
#define UIP_EXT_OPT_FIRST ((struct hbho_mcast *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN + 2])

extern uint16_t uip_slen;
#define SEQ_VAL_ADD(s, n) (((s) + (n)) % 0x8000)
#define HBH_GET_SV_MSB(h) ((h)->seq_id_msb & 0x7F)
/*---------------------------------------------------------------------------*/
static void
mcast_fwd()
{
  UIP_IP_BUF->ttl--;
  PRINTF("fcf: %u bytes.\n",uip_len);    
  tcpip_output(NULL);
  uip_clear_buf();
}
/*---------------------------------------------------------------------------*/
static uint8_t 
mcast_fwd_judge(uint16_t seqno) {
  uint8_t i = 0;

  for(i=0; i< FCF_SEQ_BUF_MAX; i++) {
    if(seqno == seq_array[i].seq_id) {
      if(seq_array[i].seq_fwd_count < FCF_FWD_MAX_COUNT) {
        /* If we enter here, we will definitely forward */
        UIP_MCAST6_STATS_ADD(mcast_fwd);
        mcast_fwd();
      }
      return UIP_MCAST6_DROP;
    }
  } 

  seq_array[next_del_p].seq_id = seqno;
  seq_array[next_del_p].seq_fwd_count = 0;
  
  /* If we enter here, we will definitely forward */
  UIP_MCAST6_STATS_ADD(mcast_fwd);
  mcast_fwd();
  seq_array[next_del_p].seq_fwd_count ++; 
  next_del_p = (next_del_p +1) % FCF_SEQ_BUF_MAX;  

  return UIP_MCAST6_ACCEPT;  
}  
/*---------------------------------------------------------------------------*/
static uint8_t
in()
{
  uint16_t seqno;

  if(UIP_IP_BUF->ttl <= 1) {
    UIP_MCAST6_STATS_ADD(mcast_dropped);
    return UIP_MCAST6_DROP;
  }
  
#if ORPL_BITMAP_DUPLICATE_DETECTION 
  seqno = get_orpl_packet_seqno();
#else
  /* Check the Next Header field: Must be HBHO */
  if(UIP_IP_BUF->proto != UIP_PROTO_HBHO) {
    PRINTF("FCF: Mcast I/O, bad proto\n");
    UIP_MCAST6_STATS_ADD(mcast_bad);
    return UIP_MCAST6_DROP;
  } else {
    /* Check the Option Type */
    if(UIP_EXT_OPT_FIRST->type != HBHO_OPT_TYPE) {
      PRINTF("FCF: Mcast I/O, bad HBHO type\n");
      UIP_MCAST6_STATS_ADD(mcast_bad);
      return UIP_MCAST6_DROP;
    }
  }

  lochbhmptr = UIP_EXT_OPT_FIRST;
  if(lochbhmptr->len != HBHO_OPT_LEN) {
    PRINTF("FCF: Mcast I/O, bad length\n");
    UIP_MCAST6_STATS_ADD(mcast_bad);
    return UIP_MCAST6_DROP;
  }

  seqno = lochbhmptr->seq_id_lsb;
  seqno |= HBH_GET_SV_MSB(lochbhmptr) << 8;
#endif
  
  if(mcast_fwd_judge(seqno) == UIP_MCAST6_DROP){
    return UIP_MCAST6_DROP;
  }

  /* Done with this packet unless we are a member of the mcast group */
  if(!uip_ds6_is_my_maddr(&UIP_IP_BUF->destipaddr)) {
    PRINTF("fcf: Not a group member. No further processing\n");
    return UIP_MCAST6_DROP;
  } else {

    receive_count +=1;

    UIP_MCAST6_STATS_ADD(mcast_in_ours);
    
    #if LOW_LATENCY
      set_low_latency_channel_clear_flag(1);
    #endif
   
    return UIP_MCAST6_ACCEPT;
  }
}
/*---------------------------------------------------------------------------*/
static void
init()
{
  UIP_MCAST6_STATS_INIT(NULL);

  uint8_t i = 0;
  for ( i=0; i< FCF_SEQ_BUF_MAX; i++)
  {
    seq_array[i].seq_id = 0xffff;
    seq_array[i].seq_fwd_count = 0;
  }
  next_del_p = 0;
  last_seq = 0;
}
/*---------------------------------------------------------------------------*/
static void
out()
{
 if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
    PRINTF("FCF: Mcast I/O, bad source\n");
    UIP_MCAST6_STATS_ADD(mcast_bad);
    goto drop;
  }

#if !ORPL_BITMAP_DUPLICATE_DETECTION
  if(uip_len + HBHO_TOTAL_LEN > UIP_BUFSIZE) {
    PRINTF("FCF: Multicast Out can not add HBHO. Packet too long\n");
    goto drop;
  }

  /* Slide 'right' by HBHO_TOTAL_LEN bytes */
  memmove(UIP_EXT_BUF_NEXT, UIP_EXT_BUF, uip_len - UIP_IPH_LEN);
  memset(UIP_EXT_BUF, 0, HBHO_TOTAL_LEN);

  UIP_EXT_BUF->next = UIP_IP_BUF->proto;
  UIP_EXT_BUF->len = 0;

  lochbhmptr = UIP_EXT_OPT_FIRST;
  lochbhmptr->type = HBHO_OPT_TYPE;
  lochbhmptr->len = HBHO_OPT_LEN;
  /* Set the sequence ID */
  last_seq = SEQ_VAL_ADD(last_seq, 1);
  lochbhmptr->seq_id_msb = last_seq >> 8;
  lochbhmptr->seq_id_lsb = last_seq & 0xFF;

  /* PadN */
  lochbhmptr->padn_type = UIP_EXT_HDR_OPT_PADN;
  lochbhmptr->padn_len = 0 ;

  uip_ext_len += HBHO_TOTAL_LEN;
  uip_len += HBHO_TOTAL_LEN;

  /* Update the proto and length field in the v6 header */
  UIP_IP_BUF->proto = UIP_PROTO_HBHO;
  UIP_IP_BUF->len[0] = ((uip_len - UIP_IPH_LEN) >> 8);
  UIP_IP_BUF->len[1] = ((uip_len - UIP_IPH_LEN) & 0xff);
#endif

  tcpip_output(NULL);
  UIP_MCAST6_STATS_ADD(mcast_out);

drop:
  uip_slen = 0;
  uip_clear_buf();
}
uint16_t get_receive_count(void){
  return receive_count;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief The fcf engine driver
 */
const struct uip_mcast6_driver fcf_driver = {
 "fcf",
  init,
  out,
  in,
};
/*---------------------------------------------------------------------------*/
/** @} */
