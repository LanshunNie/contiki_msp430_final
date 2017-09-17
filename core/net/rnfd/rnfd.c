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
#include "rnfd-icmp6.h" 
#include "multi-subnet.h"

#define DEBUG 0//DEBUG_NONE
#include "net/ip/uip-debug.h"
#include <string.h>

#define NUM_LT(a,b) (((int8_t)(((uint8_t)(a)) - ((uint8_t)(b)))) < 0)

#define RNFD_INFO_MAXNUM  3

static rnfd_info_t rnfd_info_array[RNFD_INFO_MAXNUM];
static struct ctimer rnfd_ct;
static uint8_t rnfd_offline_nodag;
static uint8_t root_immediate_children_flag;
static uint8_t rnfd_start_flag = 1;

#define RNFD_DELAY_ENTER_SUSPECT  0

static void rnfd_offline_out(void *p);
/*---------------------------------------------------------------------------*/
static clock_time_t 
random_delay(uint16_t base_time, uint16_t random_time){
  return base_time * CLOCK_SECOND + random_rand()%(random_time * CLOCK_SECOND);
}
/*---------------------------------------------------------------------------*/
static int 
rnfd_info_find_freespace(){
  int i = 0;

  for(i = 0;i < RNFD_INFO_MAXNUM; i++){
   	if(rnfd_info_array[i].rnfd_root_state == 0){
      return (i + 1);
   	}
  }
  
  for(i = 0;i < RNFD_INFO_MAXNUM; i++){
    if(rnfd_info_array[i].rnfd_root_state == RNFD_ROOT_OFFLINE_TYPE){
      return (i + 1);
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
static rnfd_info_t * 
rnfd_info_get(uint8_t instance_id,uip_ipaddr_t *dag_id){
  int i;
  for(i = 0;i < RNFD_INFO_MAXNUM; i++){
    if(rnfd_info_array[i].rnfd_instance_id == instance_id &&
      uip_ipaddr_cmp(&(rnfd_info_array[i].rnfd_dag_id),dag_id)){
      return &(rnfd_info_array[i]);
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
 
static uint8_t
rnfd_root_neighbor(uip_ipaddr_t * dag_id){
  uip_ipaddr_t tmp_ipaddr,nbr_ipaddr;

  uip_ip6addr(&tmp_ipaddr, 0xfe80, 0, 0, 0, 0, 0, 0, 0);
  memcpy(&nbr_ipaddr,dag_id,16);
  memcpy(&nbr_ipaddr,&tmp_ipaddr,8);

  uip_ds6_nbr_t *nbr = uip_ds6_nbr_lookup(&nbr_ipaddr);
  return (nbr != NULL);
}
/*---------------------------------------------------------------------------*/
#if 0
static uint8_t
rnfd_root_immediate_children(uint8_t instance_id,uip_ipaddr_t * dag_id){
  rpl_instance_t *instance;
  uip_ipaddr_t* parent_ipaddr = NULL;
  uip_ipaddr_t tmp_parent_ipaddr;

  instance = rpl_get_instance(instance_id);
  /* if instance == NULL or dag == NULL,this node is not root node .*/
  if(instance == NULL || instance->def_route == NULL){
    return 0;
  }
  
  parent_ipaddr = &(instance->def_route->ipaddr);
  memcpy(&tmp_parent_ipaddr,parent_ipaddr, sizeof(uip_ipaddr_t));
  memcpy(&tmp_parent_ipaddr,dag_id,8);

  PRINTF("rnfd parent_ipaddr:");
  PRINT6ADDR(&tmp_parent_ipaddr);
  PRINTF("\n");

  /* if satisfy , root node immediate children node. */
  return (uip_ipaddr_cmp(dag_id, &tmp_parent_ipaddr));
}
#endif
/*---------------------------------------------------------------------------*/
#if 0
static void
rnfd_info_del(void *p){
  rnfd_info_t * rnfd_info_p = (rnfd_info_t *)p;

  if(rnfd_info_p->rnfd_root_state == RNFD_ROOT_OFFLINE_TYPE){
    memset(rnfd_info_p, 0, sizeof(rnfd_info_t));
  }
}
#endif
/*---------------------------------------------------------------------------*/
static void 
rnfd_global(void *p)
{
  rnfd_info_t * rnfd_info_p = (rnfd_info_t *)p;
  if(rnfd_info_p != NULL){
    rnfd_info_p->rnfd_scope = RNFD_GLOBAL_SCOPE;
  }
}
/*---------------------------------------------------------------------------*/
static void
set_rnfd_offline_nodag(uint8_t nodag_flag){
  rnfd_offline_nodag = nodag_flag;
}
/*---------------------------------------------------------------------------*/
void
set_rnfd_start_flag(uint8_t flag){
  rnfd_start_flag = flag;
}
/*---------------------------------------------------------------------------*/
uint8_t
get_rnfd_start_flag(void){
  return rnfd_start_flag;
}
/*---------------------------------------------------------------------------*/
uint8_t
get_rnfd_offline_nodag(void)
{
#if UIP_CONF_IPV6_RNFD
  return rnfd_offline_nodag;
#else
  return 0;
#endif
}
/*---------------------------------------------------------------------------*/
static void 
rnfd_free_dag(void *p)
{
  rnfd_info_t * rnfd_info_p = (rnfd_info_t *)p;

  if(rnfd_info_p != NULL && rnfd_info_p->rnfd_root_state == RNFD_ROOT_OFFLINE_TYPE){
    rpl_instance_t * instance = rpl_get_instance(rnfd_info_p->rnfd_instance_id);
    rpl_dag_t *dag= rpl_get_dag(rnfd_info_p->rnfd_instance_id, &(rnfd_info_p->rnfd_dag_id));
    printf("free dag and send offline message\n");
     
    if(dag != NULL){
      rpl_free_dag(dag);
      //set_own_subnet_panid(0xFFFF);

      if(dag == instance->current_dag){
        printf("free current_dag ,should reselect current_dag\n");
        instance->current_dag = NULL;

        rnfd_set_root_immediate_children(0);
#if RPL_MAX_DAG_PER_INSTANCE > 1
        rpl_select_current_dag(instance);
#endif
      }
      if(instance->current_dag == NULL){
        printf("free instance\n");
        rpl_free_instance(instance);
        set_rnfd_offline_nodag(1);
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void 
rnfd_offline_out(void *p){
  rnfd_info_t * rnfd_info_p = (rnfd_info_t *)p;
  if(rnfd_info_p != NULL && rnfd_info_p->rnfd_root_state == RNFD_ROOT_SUSPECT_TYPE){
    rnfd_info_p->rnfd_root_state = RNFD_ROOT_OFFLINE_TYPE;

    ctimer_set(&(rnfd_info_p->rnfd_ctimer),random_delay(20,30), rnfd_free_dag, rnfd_info_p);

  // ctimer_set(&(rnfd_info_p ->rnfd_ctimer), random_delay(RNFD_DELETE_OFFLINE_INTERVAL,5), 
  //   rnfd_info_del, rnfd_info_p);

    rnfd_send(rnfd_info_p);
  }
}
/*---------------------------------------------------------------------------*/
static void 
rnfd_suspect_out(void *p){
  rnfd_info_t * rnfd_info_p = (rnfd_info_t *)p;
  if(rnfd_info_p != NULL && rnfd_info_p->rnfd_root_state == RNFD_ROOT_SUSPECT_TYPE){
    rnfd_send(rnfd_info_p);
    rnfd_info_p->rnfd_suspect_send_count ++;

    if(rnfd_info_p->rnfd_suspect_send_count >= RNFD_SEND_SUSPECTMSG_MAXNUM){
      rnfd_info_p->rnfd_suspect_send_count = 0;
      //ctimer_stop(&(rnfd_info_p->rnfd_ctimer));
      if(root_immediate_children_flag) {
        ctimer_set(&(rnfd_info_p->rnfd_ctimer),random_delay(RNFD_SEND_OFFLINEMSG_INTERVAL,5),
           rnfd_offline_out, rnfd_info_p);    
      }else{
        rnfd_info_p->rnfd_root_state = RNFD_ROOT_ONLINE_TYPE;
      }
    }else{
  	  ctimer_set(&(rnfd_info_p->rnfd_ctimer), random_delay(RNFD_SEND_SUSPECTMSG_BASETIME,RNFD_SEND_SUSPECTMSG_RANDOMTIME), 
  	    rnfd_suspect_out, rnfd_info_p);
    } 
  }
}
/*---------------------------------------------------------------------------*/
#if RNFD_DELAY_ENTER_SUSPECT
static void  
rnfd_delay_schdule_suspect_out(void *p){
  rnfd_info_t * rnfd_info = (rnfd_info_t *)p;

  if(rnfd_info){
    if(rnfd_info->rnfd_root_state == RNFD_ROOT_ONLINE_TYPE && rnfd_info->rnfd_scope == RNFD_GLOBAL_SCOPE){
      rnfd_info->rnfd_root_state = RNFD_ROOT_SUSPECT_TYPE;
      //ctimer_stop(&(rnfd_info->rnfd_ctimer));
      ctimer_set(&(rnfd_info->rnfd_ctimer),random_delay(20,10), rnfd_suspect_out, rnfd_info); 
    } 
  }   
}
#endif
/*---------------------------------------------------------------------------*/
void
rnfd_set_root_immediate_children(uint8_t flag){
  root_immediate_children_flag = flag;
}
/*---------------------------------------------------------------------------*/
uint8_t 
rnfd_info_enter_blacklist(uint8_t instance_id,uip_ipaddr_t *dag_id){
  rnfd_info_t * rnfd_info;
  
  rnfd_info = rnfd_info_get(instance_id,dag_id);
  if(rnfd_info){
    if(rnfd_info->rnfd_root_state == RNFD_ROOT_OFFLINE_TYPE){
      ctimer_set(&rnfd_ct,random_delay(1,10),rnfd_send, rnfd_info);
      return 1;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void 
rnfd_info_add(uint8_t instance_id,uip_ipaddr_t *dag_id){
  int rnfd_info_index =0;
  rnfd_info_t * rnfd_info;

  rnfd_info = rnfd_info_get(instance_id,dag_id);
  if(rnfd_info == NULL){
    rnfd_info_index = rnfd_info_find_freespace();
  }

  if(rnfd_info_index){
    memset(&rnfd_info_array[rnfd_info_index - 1], 0, sizeof(rnfd_info_t));
    
    rnfd_info_array[rnfd_info_index - 1].rnfd_root_state = RNFD_ROOT_ONLINE_TYPE;
    rnfd_info_array[rnfd_info_index - 1].rnfd_scope = RNFD_GLOBAL_SCOPE;
    rnfd_info_array[rnfd_info_index - 1].rnfd_suspect_send_count = 0;
  	rnfd_info_array[rnfd_info_index - 1].rnfd_instance_id = instance_id;
  	memcpy(&(rnfd_info_array[rnfd_info_index - 1].rnfd_dag_id), dag_id, sizeof(uip_ipaddr_t));
  }
  set_rnfd_offline_nodag(0);
}
/*---------------------------------------------------------------------------*/
void
rnfd_info_del_info(uint8_t instance_id,uip_ipaddr_t *dag_id) {
  rnfd_info_t * rnfd_info = rnfd_info_get(instance_id,dag_id);
  
  if(rnfd_info != NULL){
    ctimer_stop(&(rnfd_info->rnfd_ctimer));
    memset(rnfd_info, 0, sizeof(rnfd_info_t));
  }
}
/*---------------------------------------------------------------------------*/
uint8_t 
rnfd_info_get_state(rpl_dag_t *dag){
  rnfd_info_t * rnfd_info;
  
  if(dag == NULL || dag->instance == NULL){
    return 0;
  }

  rnfd_info = rnfd_info_get(dag->instance->instance_id,&(dag->dag_id));

  if(rnfd_info){ 
    return (rnfd_info->rnfd_root_state);  
  } 
  return 0;  
}
/*---------------------------------------------------------------------------*/
void 
rnfd_schdule_suspect_out(rpl_dag_t *dag){
  rnfd_info_t * rnfd_info;
  
  if(dag == NULL || dag->instance == NULL){
    return;
  }

  rnfd_info = rnfd_info_get(dag->instance->instance_id,&(dag->dag_id));

  if(rnfd_info){
  	if(rnfd_info->rnfd_root_state == RNFD_ROOT_ONLINE_TYPE) {
      rnfd_info->rnfd_root_state = RNFD_ROOT_SUSPECT_TYPE;
      //ctimer_stop(&(rnfd_info->rnfd_ctimer));
      ctimer_set(&(rnfd_info->rnfd_ctimer),random_delay(45,15), rnfd_suspect_out, rnfd_info); 
  	}	
  } 	
}
/*---------------------------------------------------------------------------*/
void 
rnfd_request_deal(uint8_t instance_id,uip_ipaddr_t *dag_id){
  rnfd_info_t * rnfd_info;
  rnfd_info = rnfd_info_get(instance_id,dag_id);
  if(rnfd_info != NULL){
    ctimer_set(&rnfd_ct,random_delay(1,16),rnfd_send, rnfd_info);
  }
}
/*---------------------------------------------------------------------------*/
void 
rnfd_common_input(rnfd_info_t * rnfd_msg){
  rnfd_info_t * rnfd_info;
  rnfd_info = rnfd_info_get(rnfd_msg->rnfd_instance_id,&(rnfd_msg->rnfd_dag_id));
  
  //printf("rnfd_msg instance_id:%02x\n",rnfd_msg->rnfd_instance_id);
  if(rnfd_info == NULL){
    return;
  }
  
  //printf("msg seqnum:%u ,info seqnum:%u\n",rnfd_msg->rnfd_seqnum,rnfd_info->rnfd_seqnum);

  if(NUM_LT(rnfd_msg->rnfd_seqnum,rnfd_info->rnfd_seqnum)){
    ctimer_set(&rnfd_ct,random_delay(1,16),rnfd_send, rnfd_info);
    return;
  }

  if(NUM_LT(rnfd_info->rnfd_seqnum,rnfd_msg->rnfd_seqnum)){
    rnfd_info->rnfd_seqnum = rnfd_msg->rnfd_seqnum; 
    rnfd_info->rnfd_scope = rnfd_msg->rnfd_scope;
    if(rnfd_msg->rnfd_root_state == RNFD_ROOT_SUSPECT_TYPE){
      if(rnfd_root_neighbor(&(rnfd_msg->rnfd_dag_id))){
        rnfd_info->rnfd_root_state = rnfd_msg->rnfd_root_state;
        //ctimer_stop(&(rnfd_info->rnfd_ctimer));
        ctimer_set(&(rnfd_info->rnfd_ctimer), random_delay(1,16), 
          rnfd_suspect_out, rnfd_info);
      }else{
        rnfd_info->rnfd_root_state = RNFD_ROOT_ONLINE_TYPE;
        rnfd_info->rnfd_scope = RNFD_GLOBAL_SCOPE;
      }
    }else {
      if(rnfd_msg->rnfd_root_state == RNFD_ROOT_OFFLINE_TYPE && 
          rnfd_info->rnfd_root_state != RNFD_ROOT_OFFLINE_TYPE){
        ctimer_set(&(rnfd_info->rnfd_ctimer),random_delay(20,30), rnfd_free_dag, rnfd_info);
      }else{
        ctimer_stop(&(rnfd_info->rnfd_ctimer));
      }
      rnfd_info->rnfd_root_state = rnfd_msg->rnfd_root_state;
      rnfd_info->rnfd_suspect_send_count = 0;     
      ctimer_set(&rnfd_ct,random_delay(1,16),rnfd_send, rnfd_info);
    }
    return;
  }
  
  if(rnfd_msg->rnfd_seqnum == rnfd_info->rnfd_seqnum){
    if(rnfd_msg->rnfd_root_state == rnfd_info->rnfd_root_state){
#if RNFD_DELAY_ENTER_SUSPECT   
      if(rnfd_info->rnfd_root_state == RNFD_ROOT_ONLINE_TYPE){
        rnfd_info->rnfd_suspect_send_count = 0; 
        rnfd_info->rnfd_scope = RNFD_GLOBAL_SCOPE;
        ctimer_stop(&(rnfd_info->rnfd_ctimer));
      }
#endif
      return;
    }

    if(rnfd_msg->rnfd_root_state == RNFD_ROOT_OFFLINE_TYPE){
      /*
      if(rnfd_root_immediate_children(rnfd_msg->rnfd_instance_id,&(rnfd_msg->rnfd_dag_id))&&
        rnfd_info->rnfd_root_state ==RNFD_ROOT_ONLINE_TYPE){
        rnfd_info->rnfd_root_state = RNFD_ROOT_SUSPECT_TYPE;
        ctimer_stop(&(rnfd_info->rnfd_ctimer));
        ctimer_set(&rnfd_ct,CLOCK_SECOND,rnfd_send, rnfd_info);
        return;
      }*/

      rnfd_info->rnfd_root_state = rnfd_msg->rnfd_root_state;
      rnfd_info->rnfd_scope = rnfd_msg->rnfd_scope;
      rnfd_info->rnfd_suspect_send_count = 0;     
      //ctimer_stop(&(rnfd_info->rnfd_ctimer));
      ctimer_set(&(rnfd_info->rnfd_ctimer), random_delay(20,30), rnfd_free_dag, rnfd_info);
      ctimer_set(&rnfd_ct,random_delay(1,16),rnfd_send, rnfd_info);
      return;
    }

    if(rnfd_info->rnfd_root_state == RNFD_ROOT_OFFLINE_TYPE){
      ctimer_set(&rnfd_ct,random_delay(1,16),rnfd_send, rnfd_info);
      return;
    }
   
    if(rnfd_msg->rnfd_root_state == RNFD_ROOT_ONLINE_TYPE && 
      rnfd_msg->rnfd_scope == RNFD_LOCAL_SCOPE &&
      rnfd_info->rnfd_root_state == RNFD_ROOT_SUSPECT_TYPE){
      
      rnfd_info->rnfd_root_state = rnfd_msg->rnfd_root_state;
      rnfd_info->rnfd_scope = rnfd_msg->rnfd_scope;
      rnfd_info->rnfd_suspect_send_count = 0; 

      ctimer_set(&(rnfd_info->rnfd_ctimer), 120*CLOCK_SECOND, rnfd_global, rnfd_info);
      ctimer_set(&rnfd_ct,random_delay(1,16),rnfd_send, rnfd_info);
      return;
    }

    if(rnfd_msg->rnfd_root_state == RNFD_ROOT_SUSPECT_TYPE && 
      rnfd_info->rnfd_root_state == RNFD_ROOT_ONLINE_TYPE &&
      rnfd_info->rnfd_scope == RNFD_LOCAL_SCOPE){ 
      ctimer_set(&rnfd_ct,random_delay(1,16),rnfd_send, rnfd_info);
      return;
    }
  
    if(rnfd_msg->rnfd_root_state == RNFD_ROOT_SUSPECT_TYPE && 
      rnfd_info->rnfd_root_state == RNFD_ROOT_ONLINE_TYPE &&
      rnfd_info->rnfd_scope == RNFD_GLOBAL_SCOPE && rnfd_root_neighbor(&(rnfd_msg->rnfd_dag_id))){ 
#if RNFD_DELAY_ENTER_SUSPECT 
      ctimer_set(&(rnfd_info->rnfd_ctimer),random_delay(20,15),rnfd_delay_schdule_suspect_out, rnfd_info);
#else
      rnfd_info->rnfd_root_state = RNFD_ROOT_SUSPECT_TYPE;
      ctimer_set(&(rnfd_info->rnfd_ctimer),random_delay(20,15),rnfd_suspect_out, rnfd_info);
#endif
    }
  } 
}
/*---------------------------------------------------------------------------*/
/* RNFD initialization */
void
rnfd_init()
{
  rnfd_offline_nodag = 0;
  memset(rnfd_info_array,0, sizeof(rnfd_info_array));
  rnfd_icmp6_register_handlers();
}
/*---------------------------------------------------------------------------*/

