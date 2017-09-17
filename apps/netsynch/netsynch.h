/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * $Id: netsynch.h,v 1.4 2008/07/01 21:02:51 adamdunkels Exp $
 */

/**
 * \file
 *         Header file for a simple time synchronization mechanism
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#ifndef __NETSYNCH_H__
#define __NETSYNCH_H__

#include "net/mac/mac.h"
#include "sys/clock.h"
#include "contiki.h"
#include "net/ip/uip.h"

struct netsynch_msg {
  uint8_t seqnum;
  uint8_t authority_level;
  soft_time caltime;
  uint8_t sche_bitmap[18];

  unsigned short nodeID; 
};


/**
 * \brief      Initialize the netsynch module
 *
 *             This function initializes the netsynch module. This
 *             function must not be called before rime_init().
 *
 */
void netsynch_init(void);

/**
 * \brief      Get the current time-synchronized time
 * \return     The current time-synchronized time
 *
 *             This function returns the current time-synchronized
 *             time.
 *
 */
void netsynch_time(void);

/**
 * \brief      Get the current authority level of the time-synchronized time
 * \return     The current authority level of the time-synchronized time
 *
 *             This function returns the current authority level of
 *             the time-synchronized time. A node with a lower
 *             authority level is defined to have a better notion of
 *             time than a node with a higher authority
 *             level. Authority level 0 is best and should be used by
 *             a sink node that has a connection to an outside,
 *             "true", clock source.
 *
 */
int netsynch_authority_level(void);

/**
 * \brief      Set the authority level of the current time
 * \param level The authority level
 */
void netsynch_set_authority_level(int level);

void schedule_bitmap_set_new(uint8_t bitmap[]);

void update_new_schedule(void*p);

// void update_schedule(uint8_t bitmap[]);
void update_schedule(void *p);

void schedule_bitmap_get(uint8_t *bitmap);

void netsynch_cal_offset(soft_time cal_time);

int netsynch_get_offset(void);

int netsynch_is_synched(void);

uint8_t get_lastseqnum(void);

void set_recvmsg(struct netsynch_msg msg);
void get_recvmsg(struct netsynch_msg *msg);

void netsync_common_timesource(void);

void netsync_start_common_request(void);

#if 1
void init_concentrator_ipv6addr(void);

uip_ipaddr_t* get_concentrator_ipv6addr(void);

void set_concentrator_ipv6addr(const uip_ipaddr_t *ip6addr);
#endif

extern  process_event_t netsynch_event;

#endif /* __NETSYNCH_H__ */

