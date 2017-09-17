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
 *	 ORPL bitmap configure.
 * \author
 *	@yang
 *
 */

#ifndef ORPL_BITMAP_CONF_H
#define ORPL_BITMAP_CONF_H

/* ORPL does not use RPL's normal downwards routing */
#undef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_NO_DOWNWARD_ROUTES

#undef RPL_MOP_DOWNWARD_STORING
#define RPL_MOP_DOWNWARD_STORING  0  //0

/* ORPL does not use traditional routing entries */
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES  0

#undef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG  0

#ifndef ORPL_MOP_DOWNWARD_STORING
#define ORPL_MOP_DOWNWARD_STORING  0
#endif

#ifndef NODE_BIT_NUM
#define NODE_BIT_NUM  8 //9
#endif

#if ORPL_MOP_DOWNWARD_STORING
#ifdef NODE_BIT_NUM
/* Routing set size (in bits) */
#define ROUTING_SET_M        (1 << NODE_BIT_NUM)
#endif /* NODE_BIT_NUM */

#ifndef ROUTING_SET_M
#define ROUTING_SET_M        512 
#endif /* ROUTING_SET_M */

#if ROUTING_SET_M > 512
#error "ROUTING_SET_M too large (max: 512)"
#endif

/* bitmap transmissions are always delayed by ORPL_BITMAP_DELAY +/- ORPL_BITMAP_DELAY/2 */
#ifdef RPL_CONF_BITMAP_DELAY
#define ORPL_BITMAP_DELAY                 RPL_CONF_BITMAP_DELAY
#else /* RPL_CONF_BITMAP_DELAY */
#define ORPL_BITMAP_DELAY                 (CLOCK_SECOND * 16)  //4
#endif /* RPL_CONF_BITMAP_DELAY */
#endif /*ORPL_MOP_DOWNWARD_STORING */

#ifndef ORPL_RANK_W
#define ORPL_RANK_W  128    //160
#endif

/* The current ORPL implementation assumes a single instance */
#define RPL_CONF_MAX_INSTANCES    1 /* default 1 */

/* ORPL runs without IPv6 NA/ND */
#undef UIP_CONF_ND6_SEND_NA
#define UIP_CONF_ND6_SEND_NA 0

/* ORPL is not compatible with sending bursts, we therefore
 * set the number of CCA before transmitting to 2 only */
#ifndef CONTIKIMAC_CONF_CCA_COUNT_MAX_TX
#define CONTIKIMAC_CONF_CCA_COUNT_MAX_TX  2  /* default 6 */  //2  //4 better ?
#endif

/* ORPL is not compatible with ContikiMAC phase-lock */
#undef CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION
#define CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION 0 

#if UIP_CONF_IPV6_ORPL_BITMAP
#define ORPL_BITMAP_DUPLICATE_DETECTION  1
#define UDP_CONF_FRAG  1
#define UDP_CONF_REASSEMBLY  0
#endif

#ifndef UDP_CONF_FRAG
#define UDP_CONF_FRAG  0
#endif

#ifndef UDP_CONF_REASSEMBLY
#define UDP_CONF_REASSEMBLY  0
#endif

#ifndef ORPL_BITMAP_DUPLICATE_DETECTION
#define ORPL_BITMAP_DUPLICATE_DETECTION  0
#endif

#ifndef ORPL_BITMAP_SUPPORT_CONTIKIMAC_BURST
#define ORPL_BITMAP_SUPPORT_CONTIKIMAC_BURST 0
#endif
 
/*no need probe */
#undef RPL_CONF_WITH_PROBING
#define RPL_CONF_WITH_PROBING           0   

#undef UIP_CONF_DS6_LL_NUD
#define UIP_CONF_DS6_LL_NUD             0

#endif /* ORPL_BITMAP_CONF_H */


