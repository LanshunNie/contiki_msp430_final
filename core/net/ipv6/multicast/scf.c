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
 * \addtogroup scf-multicast
 * @{
 */
/**
 * \file
 *    This file implements 'Stateless Multicast RPL Forwarding' (scf)
 *
 * \author
 *    George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "contiki.h"
#include "contiki-net.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "net/ipv6/multicast/uip-mcast6-route.h"
#include "net/ipv6/multicast/uip-mcast6-stats.h"
#include "net/ipv6/multicast/scf.h"
#include "net/rpl/rpl.h"
#include "net/netstack.h"
#include "ctimer.h"
#include <string.h>

#if UIP_CONF_IPV6_ORPL_BITMAP
#include "net/orpl-bitmap/orpl-bitmap.h"
#endif

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
/* Macros */
/*---------------------------------------------------------------------------*/
/* CCI */
#define SCF_FWD_DELAY()  NETSTACK_RDC.channel_check_interval()
/* Number of slots in the next 500ms */
//#define SCF_INTERVAL_COUNT  ((CLOCK_SECOND >> 2) / fwd_delay)

#define SFC_FWD_MAX_COUNT  1
#define SFC_SEQ_BUF_MAX  4

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
static struct ctimer mcast_periodic;
static uint8_t mcast_len;
static uip_buf_t mcast_buf;
static uint16_t fwd_delay;
static uint8_t fwd_spread;
static struct scf_seq_array seq_array[SFC_SEQ_BUF_MAX];
static uint8_t  next_del_p ;
static uint16_t last_seq;
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
mcast_fwd(void *p)
{
  memcpy(uip_buf, &mcast_buf, mcast_len);
  uip_len = mcast_len;
  UIP_IP_BUF->ttl--;
  tcpip_output(NULL);
  uip_clear_buf();

 // printf(" fwd  \n");
}

static void 
random_fwd_delay()
{
   fwd_delay = SCF_FWD_DELAY();   //SCF_FWD_DELAY() wei 16 .
 //  printf ("fwd_delay1 : %u \n" , fwd_delay);

    /* Finalise D: D = min(SCF_FWD_DELAY(), SCF_MIN_FWD_DELAY) */
#if SCF_MIN_FWD_DELAY
    if(fwd_delay < SCF_MIN_FWD_DELAY) {
      fwd_delay = SCF_MIN_FWD_DELAY;
    }
#endif
      /* Randomise final delay in [D , D*Spread], step D */
    //  fwd_spread = SCF_INTERVAL_COUNT * 2;  //SCF_INTERVAL_COUNT--2  4hz -1
   //   if(fwd_spread > SCF_MAX_SPREAD) {
        fwd_spread = SCF_MAX_SPREAD;
   //   }
   //   printf(" fwd_delay :  %u   ,  fwd_spread  : %u  \n" , fwd_delay,fwd_spread);
      // fwd_delay wei 128 , wei 1s .
      if(fwd_spread) {

        fwd_delay = fwd_delay * (1 + ( (random_rand()) % fwd_spread)); 
     //   fwd_delay = fwd_delay * (1 + ( RTIMER_NOW() % fwd_spread)); 
      }
    //   printf(" fwd_delay 2:  %u  \n" , fwd_delay);
}

/*---------------------------------------------------------------------------*/
static uint8_t
in()
{
 // rpl_dag_t *d;                 /* Our DODAG */
 // uip_ipaddr_t *parent_ipaddr;  /* Our pref. parent's IPv6 address */
//  const uip_lladdr_t *parent_lladdr;  /* Our pref. parent's LL address */

  /*
   * Fetch a pointer to the LL address of our preferred parent
   *
   * ToDo: This rpl_get_any_dag() call is a dirty replacement of the previous
   *   rpl_get_dag(RPL_DEFAULT_INSTANCE);
   * so that things can compile with the new RPL code. This needs updated to
   * read instance ID from the RPL HBHO and use the correct parent accordingly
   */
 // d = rpl_get_any_dag();
  
 // if(!d &&d->rank == ROOT_RANK(d->instance))
 // {
       
 // }

  /* Retrieve our preferred parent's LL address */
 // parent_ipaddr = rpl_get_parent_ipaddr(d->preferred_parent);
//  parent_lladdr = uip_ds6_nbr_lladdr_from_ipaddr(parent_ipaddr);

  /*if(parent_lladdr == NULL) {
    UIP_MCAST6_STATS_ADD(mcast_dropped);
    return UIP_MCAST6_DROP;
  } */

  /*
   * We accept a datagram if it arrived from our preferred parent, discard
   * otherwise.
   */
  //  if(memcmp(parent_lladdr, packetbuf_addr(PACKETBUF_ADDR_SENDER),
  //           UIP_LLADDR_LEN)) {
  //   PRINTF("scf: Routable in but scf ignored it\n");
  //   UIP_MCAST6_STATS_ADD(mcast_dropped);
  //   return UIP_MCAST6_DROP;
  // }  

//  uint8_t *appdata;
  uint16_t seqno;
  if(UIP_IP_BUF->ttl <= 1) {
 //   UIP_MCAST6_STATS_ADD(mcast_dropped);
    return UIP_MCAST6_DROP;
  }
  
#if ORPL_BITMAP_DUPLICATE_DETECTION 
   seqno = get_orpl_packet_seqno();
  //  memcpy(&seqno,&uip_buf[UIP_LLH_LEN +uip_ext_len+UIP_IPUDPH_LEN],2);
#else
  /* Check the Next Header field: Must be HBHO */
  if(UIP_IP_BUF->proto != UIP_PROTO_HBHO) {
  //  PRINTF("SCF: Mcast I/O, bad proto\n");
  //  UIP_MCAST6_STATS_ADD(mcast_bad);
    return UIP_MCAST6_DROP;
  } else {
    /* Check the Option Type */
    if(UIP_EXT_OPT_FIRST->type != HBHO_OPT_TYPE) {
   //   PRINTF("ROLL TM: Mcast I/O, bad HBHO type\n");
   //   UIP_MCAST6_STATS_ADD(mcast_bad);
      return UIP_MCAST6_DROP;
    }
  }
  lochbhmptr = UIP_EXT_OPT_FIRST;
  if(lochbhmptr->len != HBHO_OPT_LEN) {
    PRINTF("SCF: Mcast I/O, bad length\n");
    UIP_MCAST6_STATS_ADD(mcast_bad);
    return UIP_MCAST6_DROP;
  }

  seqno = lochbhmptr->seq_id_lsb;
  seqno |= HBH_GET_SV_MSB(lochbhmptr) << 8;
#endif

  uint8_t i = 0;
 for ( i=0; i< SFC_SEQ_BUF_MAX ;i++)
  {
     if(seqno == seq_array[i].seq_id )
     {
      
       if(seq_array[i].seq_fwd_count < SFC_FWD_MAX_COUNT)
       {
         /* If we have an entry in the mcast routing table, something with
   * a higher RPL rank (somewhere down the tree) is a group member */
 /*********************************************************************/
 //   if(uip_mcast6_route_lookup(&UIP_IP_BUF->destipaddr)) {

    /* If we enter here, we will definitely forward */
      UIP_MCAST6_STATS_ADD(mcast_fwd);

    /*
     * Add a delay (D) of at least SCF_FWD_DELAY() to compensate for how
     * contikimac handles broadcasts. We can't start our TX before the sender
     * has finished its own.
     */
       random_fwd_delay();

      /*if(fwd_delay == 0) {
           // No delay required, send it, do it now, why wait? 
         UIP_IP_BUF->ttl--;
       tcpip_output(NULL);
        UIP_IP_BUF->ttl++;        // Restore before potential upstack delivery 
        
        seq_array[i].seq_fwd_count++;

      } else {
        memcpy(&mcast_buf, uip_buf, uip_len);
        mcast_len = uip_len;
        //you que xian , wei jiance shang ci shifou guoqi .
        if(ctimer_expired (&mcast_periodic)){
          ctimer_set(&mcast_periodic, fwd_delay, mcast_fwd, NULL);  
          seq_array[i].seq_fwd_count ++; 
         }
       }
       */
         memcpy(&mcast_buf, uip_buf, uip_len);
         mcast_len = uip_len;
         //you que xian , wei jiance shang ci shifou guoqi .
        if(ctimer_expired (&mcast_periodic)){
          ctimer_set(&mcast_periodic, fwd_delay, mcast_fwd, NULL);  
          seq_array[i].seq_fwd_count ++; 
         }

         PRINTF("scf: %u bytes: fwd in %u [%u]\n",
           uip_len, fwd_delay, fwd_spread);  

         }  
         return  UIP_MCAST6_DROP;
       }   
     }

    seq_array[next_del_p].seq_id = seqno;
    seq_array[next_del_p].seq_fwd_count = 0 ;

  /* If we have an entry in the mcast routing table, something with
   * a higher RPL rank (somewhere down the tree) is a group member */
  //if(uip_mcast6_route_lookup(&UIP_IP_BUF->destipaddr)) {
    /* If we enter here, we will definitely forward */
    UIP_MCAST6_STATS_ADD(mcast_fwd);
  
    /*
     * Add a delay (D) of at least SCF_FWD_DELAY() to compensate for how
     * contikimac handles broadcasts. We can't start our TX before the sender
     * has finished its own.
     */
     random_fwd_delay(); 
      memcpy(&mcast_buf, uip_buf, uip_len);
      mcast_len = uip_len; 
      
      //fwd_delay = 0;

   //   if(fwd_delay == 0) {
      /* No delay required, send it, do it now, why wait? */
   //   UIP_IP_BUF->ttl--;
   //   tcpip_output(NULL);
      //UIP_IP_BUF->ttl++;        /* Restore before potential upstack delivery */
   // } 

      ctimer_set(&mcast_periodic, fwd_delay, mcast_fwd, NULL);       
      seq_array[next_del_p].seq_fwd_count ++; 
     PRINTF("scf: %u bytes: fwd in %u [%u]\n",
           uip_len, fwd_delay, fwd_spread);    
   
    next_del_p = (next_del_p +1) % SFC_SEQ_BUF_MAX ;
  //  printf ("next_del_p: %u \n" ,next_del_p);

  /* Done with this packet unless we are a member of the mcast group */
  if(!uip_ds6_is_my_maddr(&UIP_IP_BUF->destipaddr)) {
    PRINTF("scf: Not a group member. No further processing\n");
    return UIP_MCAST6_DROP;
  } else {
    PRINTF("scf: Ours. Deliver to upper layers\n");
    UIP_MCAST6_STATS_ADD(mcast_in_ours);
    return UIP_MCAST6_ACCEPT;
  }
}
/*---------------------------------------------------------------------------*/
static void
init()
{
  UIP_MCAST6_STATS_INIT(NULL);

  //uip_mcast6_route_init();
  uint8_t i=0;
  for ( i=0; i< SFC_SEQ_BUF_MAX ; i++)
  {
    seq_array[i].seq_id = 0xffff ;
    seq_array[i].seq_fwd_count = 0 ;
  }
  next_del_p = 0;
  last_seq = 0;
}
/*---------------------------------------------------------------------------*/
static void
out()
{
 if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
    PRINTF("SCF: Mcast I/O, bad source\n");
    UIP_MCAST6_STATS_ADD(mcast_bad);
    goto drop;
  }

#if !ORPL_BITMAP_DUPLICATE_DETECTION
  if(uip_len + HBHO_TOTAL_LEN > UIP_BUFSIZE) {
    PRINTF("ROLL TM: Multicast Out can not add HBHO. Packet too long\n");
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
  uip_len = 0;
  uip_ext_len = 0;
 
}
/*---------------------------------------------------------------------------*/
/**
 * \brief The scf engine driver
 */
const struct uip_mcast6_driver scf_driver = {
 "scf",
  init,
  out,
  in,
};
/*---------------------------------------------------------------------------*/
/** @} */
