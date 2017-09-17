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
 * $Id: wake-common.h,v 1.4 2016/10/06 19:36:15 
 */

/**
 * \file
 *         Header file for wake 
 * \author
 *         @yang
 */

#ifndef WAKE_COMMON_H_
#define WAKE_COMMON_H_

#include "contiki.h"
#define WAKE_NODE_TYPE  0x1234
#define MAX_PACKET_LEN   30
 
struct wake_st{
  uint8_t wake_buf[MAX_PACKET_LEN];
  uint8_t wake_len;
  uint8_t wake_collisions;
  uint8_t wake_seq;
};

void cache_wake_info(uint8_t seq,uint8_t buf[],uint8_t len);
void wake_sent_callback(int status, int num_transmissions);
struct wake_st *get_wake_info(void);

void set_wake_send(uint8_t flag);
uint8_t get_wake_send(void);
void wake_send(void *ptr);
void wake_common_init(void);

#endif /* WAKE_COMMON_H_ */

