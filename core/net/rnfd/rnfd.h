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
 *	Public API declarations for RNFD.   // root node fail detect
 * \author
 *	@yang
 *
 */

#ifndef RNFD_H_
#define RNFD_H_

#include "rnfd-conf.h"
#include "rnfd-icmp6.h"
#include "rnfd-root.h" 
 
#include "contiki.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"

/*notice : rnfd_valid_flag is in order to avoid frequent suspect send, also avoid some node
    to cause suspect message send storm ,because they don't receive root online message .*/
/* if rnfd_valid_flag set 1,root online is valid; if rnfd_valid_flag set 0, root online is out of date.*/
/* root online state have valid lifetime ,default is 5 minutes .*/

/* root offline state mean root dodag enter blacklist .*/

struct rnfd_info_s{
   uint8_t rnfd_root_state;
   uint8_t rnfd_seqnum;
   uint8_t rnfd_scope;
   uint8_t rnfd_instance_id;
   uip_ipaddr_t rnfd_dag_id;

  // uint8_t rnfd_valid_flag;      
   uint8_t rnfd_suspect_send_count;
   struct ctimer rnfd_ctimer;
};

typedef struct rnfd_info_s rnfd_info_t;

uint8_t rnfd_info_enter_blacklist(uint8_t instance_id,uip_ipaddr_t *dag_id);
void rnfd_request_deal(uint8_t instance_id,uip_ipaddr_t *dag_id);
void rnfd_info_add(uint8_t instance_id,uip_ipaddr_t *dag_id);
void rnfd_info_del_info(uint8_t instance_id,uip_ipaddr_t *dag_id);
void rnfd_schdule_suspect_out(rpl_dag_t *dag);

void rnfd_set_root_immediate_children(uint8_t flag);

void set_rnfd_start_flag(uint8_t flag);
uint8_t get_rnfd_start_flag(void);

uint8_t rnfd_info_get_state(rpl_dag_t *dag);

uint8_t get_rnfd_offline_nodag(void);

void rnfd_common_input(rnfd_info_t * rnfd_msg);

/* RNFD initialization */
void rnfd_init();
#endif /* RNFD_H_ */


