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

#include "contiki-conf.h"
#include "sys/node-id.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "random.h"
 
#include "rnfd.h"
#include "rnfd-root.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"
#include <string.h>

#define NUM_LT(a,b) (((int8_t)(((uint8_t)(a)) - ((uint8_t)(b)))) < 0)

static struct ctimer rnfd_root_ct;
static uint8_t rnfd_seqnum =0;
static uint8_t init_seqnum_flag =0;
static rnfd_info_t rnfd_info;
/*---------------------------------------------------------------------------*/
static void
rnfd_set_seqnum(uint8_t seq){
  init_seqnum_flag =1;
  rnfd_seqnum = seq;
}
/*---------------------------------------------------------------------------*/
static void 
rnfd_update_seqnum(void){
   ++rnfd_seqnum;
}
/*---------------------------------------------------------------------------*/
#if 0
static void
rnfd_init_seq(void *p){
  if(!init_seqnum_flag){
	init_seqnum_flag =1;
	rnfd_seqnum = random_rand();
  }
}
#endif
/*---------------------------------------------------------------------------*/
void
rnfd_request_info(uint8_t instance_id,uip_ipaddr_t *dag_id){
  rnfd_request_out(instance_id,dag_id);
  //ctimer_set(&rnfd_root_ct,60 * CLOCK_SECOND,rnfd_init_seq, NULL);
}
/*---------------------------------------------------------------------------*/
void
rnfd_root_input(rnfd_info_t * rnfd_info_p){
  memcpy(&rnfd_info,rnfd_info_p, sizeof(rnfd_info_t));

  if((!init_seqnum_flag)||(NUM_LT(rnfd_seqnum,rnfd_info.rnfd_seqnum))){
    rnfd_set_seqnum(rnfd_info.rnfd_seqnum);
  }

  if(rnfd_info.rnfd_root_state == RNFD_ROOT_SUSPECT_TYPE){
    rnfd_info.rnfd_root_state = RNFD_ROOT_ONLINE_TYPE;
    rnfd_info.rnfd_seqnum = rnfd_seqnum;
    rnfd_info.rnfd_scope = RNFD_LOCAL_SCOPE;
    ctimer_set(&rnfd_root_ct,2*CLOCK_SECOND,rnfd_send, &rnfd_info);
  }else if(rnfd_info.rnfd_root_state == RNFD_ROOT_OFFLINE_TYPE){
  	if(!(NUM_LT(rnfd_info.rnfd_seqnum,rnfd_seqnum))){
      rnfd_update_seqnum();
    }

  	rnfd_info.rnfd_root_state = RNFD_ROOT_ONLINE_TYPE;
    rnfd_info.rnfd_seqnum = rnfd_seqnum;
    rnfd_info.rnfd_scope = RNFD_GLOBAL_SCOPE;
    ctimer_set(&rnfd_root_ct,CLOCK_SECOND,rnfd_send, &rnfd_info);
  }
}
/*---------------------------------------------------------------------------*/






