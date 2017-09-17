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
 */

/**
 * \file
 *         ICMP6 I/O for RNFD control messages.
 *  author @yang
 */
/**
 * \addtogroup uip6
 * @{
 */

#include "rnfd-icmp6.h"
#include "rnfd.h"  
#include "net/ip/tcpip.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-nd6.h"
#include "net/ipv6/uip-icmp6.h"
#include "net/rpl/rpl-private.h"
#include "net/packetbuf.h"
#include "net/ipv6/multicast/uip-mcast6.h"

#include <limits.h>
#include <string.h>
 
#define DEBUG 0
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF       ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_ICMP_BUF     ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])
#define UIP_ICMP_PAYLOAD ((unsigned char *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define BUFFER_MAX_LEN   40
/*---------------------------------------------------------------------------*/
static void rnfd_request_input(void);
static void rnfd_input(void);
static void rnfd_output(uint8_t data_len, uint8_t *data);
/*---------------------------------------------------------------------------*/
/* Initialise RPL bitmap ICMPv6 message handlers */
UIP_ICMP6_HANDLER(rnfd_msg_handler, ICMP6_RNFD,RNFD_MSG_CODE,rnfd_input);
UIP_ICMP6_HANDLER(rnfd_request_handler, ICMP6_RNFD,RNFD_REQUEST_CODE,rnfd_request_input);
/*---------------------------------------------------------------------------*/
static void 
rnfd_request_input(void){
  unsigned char *buffer;
  int pos;
  uint8_t rnfd_msgtype;
  uint8_t instance_id;
  uip_ipaddr_t dag_id;

  buffer = UIP_ICMP_PAYLOAD;
  pos = 0;

  rnfd_msgtype = buffer[pos++];
  instance_id  = buffer[pos++];
  memcpy(&dag_id, buffer + pos, sizeof(uip_ipaddr_t));

  if(rnfd_msgtype == RNFD_ROOT_REQUEST){
    rnfd_request_deal(instance_id,&dag_id);
  }
  
  uip_clear_buf();  
}
/*---------------------------------------------------------------------------*/
void
rnfd_request_out(uint8_t instance_id,uip_ipaddr_t *dag_id){
  int pos;
  unsigned char *buff;
  uip_ipaddr_t addr;

  pos =0;
  buff = UIP_ICMP_PAYLOAD;
  
  buff[pos++] = RNFD_ROOT_REQUEST;
  buff[pos++] = instance_id;
  memcpy(buff + pos,dag_id, sizeof(uip_ipaddr_t));
  pos += sizeof(uip_ipaddr_t);
 
  uip_create_linklocal_rplnodes_mcast(&addr);
  PRINTF("RNFD:Root sending RNFD reuest message to: ");
  PRINT6ADDR(&addr);
  PRINTF("\n");

  uip_icmp6_send(&addr,ICMP6_RNFD, RNFD_REQUEST_CODE,pos);
}
/*---------------------------------------------------------------------------*/
static void 
rnfd_input(void)
{
  unsigned char *buffer;
  int pos;
  rnfd_info_t rnfd_msg;
 
  buffer = UIP_ICMP_PAYLOAD;
  pos = 0;
  memset(&rnfd_msg , 0, sizeof(rnfd_msg));

  rnfd_msg.rnfd_root_state = buffer[pos++];
  rnfd_msg.rnfd_seqnum = buffer[pos++];
  rnfd_msg.rnfd_scope = buffer[pos++];
  rnfd_msg.rnfd_instance_id  = buffer[pos++];
  memcpy(&(rnfd_msg.rnfd_dag_id), buffer + pos, sizeof(uip_ipaddr_t));
  
  //printf("RNFD: Receive root state is: %02x, seqnum:%u\n",rnfd_msg.rnfd_root_state,rnfd_msg.rnfd_seqnum);
  PRINTF("RNFD: Receive RNFD message root state is: %02x\n",rnfd_msg.rnfd_root_state);

#if 0
#if ROOTNODE 
  //rpl_instance_t *instance;
  rpl_dag_t *dag;

  //instance = rpl_get_instance(rnfd_msg.rnfd_instance_id);
  dag = rpl_get_dag(rnfd_msg.rnfd_instance_id, &(rnfd_msg.rnfd_dag_id));

  if(dag != NULL){
    /* root node */
   // if(dag->rank == ROOT_RANK(instance)){
      rnfd_root_input(&rnfd_msg);
    //  uip_clear_buf();  
    // return;
   //}
  }
#else
  rnfd_common_input(&rnfd_msg);
#endif
#endif

#if 1
  rpl_instance_t *instance;
  rpl_dag_t *dag;

  instance = rpl_get_instance(rnfd_msg.rnfd_instance_id);
  dag = rpl_get_dag(rnfd_msg.rnfd_instance_id, &(rnfd_msg.rnfd_dag_id));

  if(dag != NULL){
    /* root node */
    if(dag->rank == ROOT_RANK(instance)){
      rnfd_root_input(&rnfd_msg);
      uip_clear_buf();  
      return;
    }
  }

  rnfd_common_input(&rnfd_msg);
#endif

  uip_clear_buf();  
}
/*---------------------------------------------------------------------------*/
/* multicast rnfd message to our neighbor */
static void
rnfd_output(uint8_t data_len, uint8_t *data){
  unsigned char *buffer;
  uip_ipaddr_t addr;

  buffer = UIP_ICMP_PAYLOAD;
  memcpy(buffer,data, data_len);

  uip_create_linklocal_rplnodes_mcast(&addr);
  PRINTF("RNFD: Sending RNFD message to: ");
  PRINT6ADDR(&addr);
  PRINTF("\n");
  
  uip_icmp6_send(&addr,ICMP6_RNFD, RNFD_MSG_CODE, data_len);
}
/*---------------------------------------------------------------------------*/
void
rnfd_send(void *p){
  int pos;
  uint8_t buff[BUFFER_MAX_LEN];
  
  rnfd_info_t * rnfd_info = (rnfd_info_t *)(p);
  memset(buff,0,BUFFER_MAX_LEN);
  pos =0;

  buff[pos++] = rnfd_info->rnfd_root_state;
  buff[pos++] = rnfd_info->rnfd_seqnum;
  buff[pos++] = rnfd_info->rnfd_scope;
  buff[pos++] = rnfd_info->rnfd_instance_id;

  memcpy(buff + pos,&(rnfd_info->rnfd_dag_id), sizeof(uip_ipaddr_t));
  pos += sizeof(uip_ipaddr_t);

  printf("RNFD: Send root state is: %02x, seqnum:%u\n",rnfd_info->rnfd_root_state,rnfd_info->rnfd_seqnum);
  
  PRINTF("RNFD: Send RNFD message root state is: %02x\n",rnfd_info->rnfd_root_state);
  rnfd_output(pos,buff);
  
  if(rnfd_info->rnfd_scope == RNFD_LOCAL_SCOPE){
    rnfd_info->rnfd_scope = RNFD_GLOBAL_SCOPE;
  }
}
/*---------------------------------------------------------------------------*/
void
rnfd_icmp6_register_handlers()
{
  uip_icmp6_register_input_handler(&rnfd_msg_handler);
  uip_icmp6_register_input_handler(&rnfd_request_handler);
}
/*---------------------------------------------------------------------------*/



