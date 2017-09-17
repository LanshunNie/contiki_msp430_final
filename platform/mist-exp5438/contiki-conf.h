/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

#ifdef PLATFORM_CONF_H
#include PLATFORM_CONF_H
#else
#include "platform-conf.h"
#endif /* PLATFORM_CONF_H */

#ifndef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     csma_driver
//#define NETSTACK_CONF_MAC     nullmac_driver
#endif /* NETSTACK_CONF_MAC */

#ifndef NETSTACK_CONF_RDC
#if COOJA_SIMULATION 
#define NETSTACK_CONF_RDC     nullrdc_driver
#else
#define NETSTACK_CONF_RDC     contikimac_driver
#endif /* COOJA_SIMULATION */
#endif /* NETSTACK_CONF_RDC */

#ifndef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE    4
#endif /* NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE */

#ifdef NETSTACK_CONF_RDC_CHANNEL_CHECK_INTERVAL
#define NETSTACK_RDC_CHANNEL_CHECK_INTERVAL  NETSTACK_CONF_RDC_CHANNEL_CHECK_INTERVAL
#else
#define NETSTACK_RDC_CHANNEL_CHECK_INTERVAL   30
#endif

#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO   cc1120_driver
#endif /* NETSTACK_CONF_RADIO */

#ifndef  RF_CHANNEL
#define  RF_CHANNEL  6
#endif

#ifndef SLIP_CONF_CRC_ON
#define SLIP_CONF_CRC_ON  0
#endif
/*------------------------------------------------------*/
/* nullrdc config*/
#define NULLRDC_CONF_802154_AUTOACK 1
/*------------------------------------------------------*/
/* contikimac config*/
#define CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION 0
#define CONTIKIMAC_CONF_CCA_CHECK_TIME		RTIMER_ARCH_SECOND/1600
#define CONTIKIMAC_CONF_CCA_COUNT_MAX		2
#define RDC_CONF_HARDWARE_CSMA 0
#define RDC_CONF_HARDWARE_ACK 0

#ifndef CONTIKIMAC_CONF_INTER_PACKET_INTERVAL
#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL		RTIMER_ARCH_SECOND/625	/* ~1.6ms */    //280

//#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL		RTIMER_ARCH_SECOND/300	/* ~2.5ms */    //280

//#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL		RTIMER_ARCH_SECOND/278	/* ~2.5ms */    //280

//#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL		RTIMER_ARCH_SECOND/275	/* ~2.5ms */    //280
#endif

#define CONTIKIMAC_CONF_CCA_SLEEP_TIME     RTIMER_ARCH_SECOND/275 //275 ~3.636ms  /* 140 = ~7.1ms, 286 = ~3.5ms */
#define CONTIKIMAC_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED    RTIMER_ARCH_SECOND/25	/* ~50ms */  //20
#define CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME (RTIMER_SECOND / 1000)
#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT 0
//#define CONTIKIMAC_CONF_SHORTEST_PACKET_SIZE 36

/*------------------------------------------------------*/
/* simplify code config*/

/*no need probe , to save energy*/
#undef RPL_CONF_WITH_PROBING
#define RPL_CONF_WITH_PROBING      0  

#ifndef RPL_MOP_DOWNWARD_STORING
#define RPL_MOP_DOWNWARD_STORING   0
#endif

#if !RPL_MOP_DOWNWARD_STORING
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES   0
#endif
 
//ping
#ifndef UIP_CONF_SUPPORT_ECHO    
#define UIP_CONF_SUPPORT_ECHO  0
#endif

//if send icmpv6 error
#ifndef UIP_CONF_SUPPORT_ICMP6_ERROR_OUTPUT
#define UIP_CONF_SUPPORT_ICMP6_ERROR_OUTPUT	  0
#endif
/*-------------------------------------------------------*/
/* root node fail detect config (RNFD) */

#ifndef UIP_CONF_IPV6_RNFD
#define UIP_CONF_IPV6_RNFD    1
#endif  /* UIP_CONF_IPV6_RNFD */

/* RNFD root offline state, node auto enter sleep config */
#if UIP_CONF_IPV6_RNFD
#ifndef UIP_CONF_AUTO_SLEEP
#define UIP_CONF_AUTO_SLEEP   1
#endif  /* UIP_CONF_AUTO_SLEEP */
#endif

#ifndef RNFD_TEST
#define RNFD_TEST    0
#endif  /* RNFD_TEST */

#if RNFD_TEST
#define UIP_CONF_STATISTICS   1
#define RPL_CONF_STATS        1
#endif
/*-------------------------------------------------------*/
/* one large network to multi subnet. */
#ifndef UIP_CONF_MULTI_SUBNET
#define UIP_CONF_MULTI_SUBNET  0
#endif  /* UIP_CONF_MULTI_SUBNET */

#if UIP_CONF_MULTI_SUBNET
#include "multi-subnet-conf.h"
#endif
/*-------------------------------------------------------*/
/* orpl bitmap config */

#ifndef UIP_CONF_IPV6_ORPL_BITMAP
#define UIP_CONF_IPV6_ORPL_BITMAP   1
#endif  /* UIP_CONF_IPV6_ORPL_BITMAP */

#if UIP_CONF_IPV6_ORPL_BITMAP
#define orpl_bitmap_contains_ipv6(ipv6)  orpl_routing_set_contains_ipv6((ipv6))
#define orpl_bitmap_fwd(lladdr)   orpl_bitmap_satisfy_fwd(lladdr)
#define orpl_bitmap_trickle_max  4  //2

#undef CONTIKIMAC_CONF_CCA_COUNT_MAX_TX
#define CONTIKIMAC_CONF_CCA_COUNT_MAX_TX  CONTIKIMAC_CONF_CCA_COUNT_MAX

#if SUBNET_PANID_CONF_LIMIT
#define ORPL_BITMAP_PANID_LIMIT  0
#else
#define ORPL_BITMAP_PANID_LIMIT  1  // 0
#endif

#define ORPL_BITMAP_LIMIT_NETWORKING  0 //1
#else
#define orpl_bitmap_contains_ipv6(ipv6)  0
#define orpl_bitmap_fwd(lladdr)  0 
#endif 

#if ORPL_BITMAP_PANID_LIMIT  
#define orpl_panid_limit(dest_pid)  orpl_bitmap_panid_limit((dest_pid))
#else
#define orpl_panid_limit(dest_pid)  1
#endif

#if ORPL_BITMAP_LIMIT_NETWORKING
#define orpl_bitmap_limit(lladdr)  orpl_bitmap_limit_networking((lladdr))
#else
#define orpl_bitmap_limit(lladdr)  1
#endif
/*-------------------------------------------------------*/
/* netstack config */
/*-------------------------------------------------------*/

/* netstack use ipv6 config */
#ifndef NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_WITH_IPV6   1
#endif 
/*-------------------------------------------------------*/
/* netstack encrypt config */

#define NETSTACK_CONF_WITH_ENCRYPT  0

#if NETSTACK_CONF_WITH_ENCRYPT
#define NETSTACK_CONF_LLSEC noncoresec_driver
#define NETSTACK_CONF_FRAMER  noncoresec_framer
#ifndef LLSEC802154_CONF_SECURITY_LEVEL
#define LLSEC802154_CONF_SECURITY_LEVEL   FRAME802154_SECURITY_LEVEL_MIC_32
//#undef  CONTIKIMAC_CONF_INTER_PACKET_INTERVAL
//#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL		RTIMER_ARCH_SECOND/625  //615
#endif /* LLSEC802154_CONF_SECURITY_LEVEL */
#endif

#ifndef AES_128_CONF   
#define AES_128_CONF aes_128_driver
#endif   
/*-------------------------------------------------------*/
/* netstack framer config */

#ifndef NETSTACK_CONF_FRAMER
#if NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_FRAMER  framer_802154
#else /* NETSTACK_CONF_WITH_IPV6 */
#define NETSTACK_CONF_FRAMER  contikimac_framer
#endif /* NETSTACK_CONF_WITH_IPV6 */
#endif /* NETSTACK_CONF_FRAMER */
/*-------------------------------------------------------*/
/* netstack ipv6/ipv4 config */

#if NETSTACK_CONF_WITH_IPV6
/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK sicslowpan_driver

#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                8
#endif

#define LINKADDR_CONF_SIZE              8

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0

#define UIP_CONF_ROUTER                 1

/* configure number of neighbors and routes */
#ifndef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS     60
#endif /* NBR_TABLE_CONF_MAX_NEIGHBORS */
#ifndef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES   20
#endif /* UIP_CONF_MAX_ROUTES */

#define UIP_CONF_ND6_SEND_RA		0
#define UIP_CONF_ND6_SEND_NA		0
#define UIP_CONF_ND6_REACHABLE_TIME     6000000
#define UIP_CONF_ND6_RETRANS_TIMER      10000

#ifndef UIP_CONF_IPV6_QUEUE_PKT
#define UIP_CONF_IPV6_QUEUE_PKT         0
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_ND6_MAX_PREFIXES       3
#define UIP_CONF_ND6_MAX_DEFROUTERS     2
#define UIP_CONF_IP_FORWARD             0

#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE		240
#endif

/* Specify a minimum packet size for 6lowpan compression to be
   enabled. This is needed for ContikiMAC, which needs packets to be
   larger than a specified size, if no ContikiMAC header should be
   used. */
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD   63  //76  //63  
#define SICSLOWPAN_CONF_COMPRESSION_IPV6        0
#define SICSLOWPAN_CONF_COMPRESSION_HC1         1
#define SICSLOWPAN_CONF_COMPRESSION_HC06        2
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06

#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1
#endif /* SICSLOWPAN_CONF_FRAG */

#ifndef SICSLOWPAN_CONF_MAXAGE
#define SICSLOWPAN_CONF_MAXAGE                  8  
#endif

#define SICSLOWPAN_CONF_CONVENTIONAL_MAC	1
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2
#ifndef SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS
#define SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS   5  //5
#endif /* SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS */

#else /* NETSTACK_CONF_WITH_IPV6 */
#define UIP_CONF_IP_FORWARD      1
#define UIP_CONF_BUFFER_SIZE     108

/* Network setup for non-IPv6 (rime). */

#define NETSTACK_CONF_NETWORK rime_driver

#define COLLECT_CONF_ANNOUNCEMENTS       1
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0
#define CONTIKIMAC_CONF_ANNOUNCEMENTS    0

#define CONTIKIMAC_CONF_COMPOWER         1
#define XMAC_CONF_COMPOWER               1
#define CXMAC_CONF_COMPOWER              1

#ifndef COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS
#define COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS     32
#endif /* COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS */

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                16
#endif /* QUEUEBUF_CONF_NUM */

#ifndef TIMESYNCH_CONF_ENABLED
#define TIMESYNCH_CONF_ENABLED           0
#endif /* TIMESYNCH_CONF_ENABLED */

#if TIMESYNCH_CONF_ENABLED
/* CC2420 SDF timestamps must be on if timesynch is enabled. */
#undef CC2420_CONF_SFD_TIMESTAMPS
#define CC2420_CONF_SFD_TIMESTAMPS       1
#endif /* TIMESYNCH_CONF_ENABLED */

#endif /* NETSTACK_CONF_WITH_IPV6 */

/* netstack ipv6/ipv4 config end */
/*-------------------------------------------------------*/

#define PACKETBUF_CONF_ATTRS_INLINE 1

#define IEEE802154_CONF_PANID       0xABCD

#define SHELL_VARS_CONF_RAM_BEGIN 0x1100
#define SHELL_VARS_CONF_RAM_END 0x2000

#define PROFILE_CONF_ON 0
#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON 1
#endif /* ENERGEST_CONF_ON */

#define ELFLOADER_CONF_TEXT_IN_ROM 0
#ifndef ELFLOADER_CONF_DATAMEMORY_SIZE
#define ELFLOADER_CONF_DATAMEMORY_SIZE 0x400
#endif /* ELFLOADER_CONF_DATAMEMORY_SIZE */
#ifndef ELFLOADER_CONF_TEXTMEMORY_SIZE
#define ELFLOADER_CONF_TEXTMEMORY_SIZE 0x800
#endif /* ELFLOADER_CONF_TEXTMEMORY_SIZE */

#define AODV_COMPLIANCE
#define AODV_NUM_RT_ENTRIES 32

#define WITH_ASCII 1

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 1
/*#define PROCESS_CONF_FASTPOLL    4*/

#define UIP_CONF_ICMP_DEST_UNREACH 1
#define UIP_CONF_DHCP_LIGHT
#ifndef  UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  100
#endif
#ifndef  UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS         100
#endif
#define UIP_CONF_MAX_CONNECTIONS 4
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
#define UIP_ARCH_IPCHKSUM        0
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0

#define UIP_CONF_TCP_SPLIT       0

#define RIMESTATS_CONF_ENABLED   0  // 1

/* include the project config */
/* PROJECT_CONF_H might be defined in the project Makefile */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */

#endif /* CONTIKI_CONF_H */



