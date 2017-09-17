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
 */


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
 * \file
 *         Project specific configuration defines for the RPl multicast
 *         example.
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#include "net/ipv6/multicast/uip-mcast6-engines.h"

/* Change this to switch engines. Engine codes in uip-mcast6-engines.h */
#define UIP_MCAST6_CONF_ENGINE  UIP_MCAST6_ENGINE_FCF

/* For Imin: Use 16 over NullRDC, 64 over Contiki MAC */
#define ROLL_TM_CONF_IMIN_1         64

#undef UIP_CONF_IPV6_RPL
#undef UIP_CONF_ND6_SEND_RA
#undef UIP_CONF_ROUTER
#define UIP_CONF_ND6_SEND_RA           0
#define UIP_CONF_ROUTER                1
#define UIP_MCAST6_ROUTE_CONF_ROUTES   0

#undef UIP_CONF_TCP
#define UIP_CONF_TCP 0

/* Code/RAM footprint savings so that things will fit on our device */
#undef UIP_CONF_DS6_NBR_NBU
#undef UIP_CONF_DS6_ROUTE_NBU
#define UIP_CONF_DS6_NBR_NBU        10
#define UIP_CONF_DS6_ROUTE_NBU      10


#if UIP_MCAST6_CONF_ENGINE == UIP_MCAST6_ENGINE_FCF
#ifndef LOW_LATENCY
#define LOW_LATENCY  0     
#endif

#undef  NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     contikimac_fcf_driver

#undef  NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE  
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE    4

#undef  CONTIKIMAC_CONF_CCA_COUNT_MAX
#define CONTIKIMAC_CONF_CCA_COUNT_MAX    2

#undef CONTIKIMAC_CONF_CCA_COUNT_MAX_TX
#define CONTIKIMAC_CONF_CCA_COUNT_MAX_TX   CONTIKIMAC_CONF_CCA_COUNT_MAX

//#undef  CONTIKIMAC_CONF_INTER_PACKET_INTERVAL	
//#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL     (CONTIKIMAC_CONF_CCA_SLEEP_TIME - 1)


#undef CONTIKIMAC_CONF_MAX_SILENCE_PERIODS
#define CONTIKIMAC_CONF_MAX_SILENCE_PERIODS     (CONTIKIMAC_CONF_CCA_COUNT_MAX + 4)

#undef CONTIKIMAC_CONF_MAX_NONACTIVITY_PERIODS
#define CONTIKIMAC_CONF_MAX_NONACTIVITY_PERIODS  (CONTIKIMAC_CONF_CCA_COUNT_MAX + 8)


//from multicat packet input to cca before tx send, program delay 45 ticks.
//do one cca delay 43 ticks. we need 88 ticks in trxeb1120 with use fcf.

#if CONTIKIMAC_CONF_CCA_COUNT_MAX == 2
#define FCF_INTER_PACKET_INTERVAL  (CONTIKIMAC_CONF_CCA_SLEEP_TIME - CONTIKIMAC_CONF_CCA_CHECK_TIME)
#else
#define FCF_CCA_SLEEP_TIME  (CONTIKIMAC_CONF_CCA_COUNT_MAX - 1) * CONTIKIMAC_CONF_CCA_SLEEP_TIME
#define FCF_CCA_CHECK_TIME  (CONTIKIMAC_CONF_CCA_COUNT_MAX - 2) * CONTIKIMAC_CONF_CCA_CHECK_TIME
#define FCF_INTER_PACKET_INTERVAL	 (FCF_CCA_SLEEP_TIME + FCF_CCA_CHECK_TIME)
#endif

#define FCF_CHECK_TIME  (CONTIKIMAC_CONF_CCA_COUNT_MAX - 2) * (CONTIKIMAC_CONF_CCA_SLEEP_TIME + CONTIKIMAC_CONF_CCA_CHECK_TIME)
//#define FCF_INTER_PACKET_INTERVAL	(CONTIKIMAC_CONF_INTER_PACKET_INTERVAL + FCF_CHECK_TIME)

//#define FCF_INTER_PACKET_INTERVAL  ((CONTIKIMAC_CONF_CCA_COUNT_MAX - 1) * CONTIKIMAC_CONF_INTER_PACKET_INTERVAL)

#define FCF_ACK_WAIT_TIME    CONTIKIMAC_CONF_INTER_PACKET_INTERVAL   // 

#if FCF_INTER_PACKET_INTERVAL > FCF_ACK_WAIT_TIME
#define FCF_REMAIN_INTER_PACKET_INTERVAL   (FCF_INTER_PACKET_INTERVAL - FCF_ACK_WAIT_TIME)
#else
#define FCF_REMAIN_INTER_PACKET_INTERVAL   0
#endif

/*----------------fcf optimize energy config-------------------*/
//2 * CONTIKIMAC_CONF_CCA_CHECK_TIME + CONTIKIMAC_CONF_CCA_SLEEP_TIME
#if FCF_INTER_PACKET_INTERVAL > (3 * CONTIKIMAC_CONF_CCA_CHECK_TIME)
#define FCF_OPTIMIZE_ENERGY   0    // 1
#else
#define FCF_OPTIMIZE_ENERGY   0     // must 0
#endif

#ifndef FCF_OPTIMIZE_ENERGY 
#define FCF_OPTIMIZE_ENERGY   0
#endif

#if FCF_OPTIMIZE_ENERGY
#if FCF_REMAIN_INTER_PACKET_INTERVAL > (3 * CONTIKIMAC_CONF_CCA_CHECK_TIME)
#define FCF_REMAIN_OPTIMIZE_ENERGY    0   //or 1
#else
#define FCF_REMAIN_OPTIMIZE_ENERGY    0   // must 0
#endif
#endif /* FCF_OPTIMIZE_ENERGY */

#ifndef FCF_REMAIN_OPTIMIZE_ENERGY 
#define FCF_REMAIN_OPTIMIZE_ENERGY   0
#endif
/*-------------------------------------------------------------*/

#undef CONTIKIMAC_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED
#define CONTIKIMAC_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED   (FCF_INTER_PACKET_INTERVAL + RTIMER_ARCH_SECOND/32)

#endif /* UIP_MCAST6_CONF_ENGINE == UIP_MCAST6_ENGINE_FCF */


#ifndef UIP_FALLBACK_INTERFACE
#define UIP_FALLBACK_INTERFACE rpl_interface
#endif

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM          4
#endif

#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE    140
#endif

#ifndef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  60
#endif

#ifndef WEBSERVER_CONF_CFS_CONNS
#define WEBSERVER_CONF_CFS_CONNS 2
#endif

#endif /* PROJECT_CONF_H_ */
