/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 *         Module for sending UDP packets through uIP.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki-conf.h"

extern uint16_t uip_slen;

#include "net/ip/uip-udp-packet.h"
#include "net/ipv6/multicast/uip-mcast6.h"

#if UIP_CONF_IPV6_ORPL_BITMAP
#include "net/orpl-bitmap/orpl-bitmap.h"
#if ORPL_BITMAP_DUPLICATE_DETECTION
#define ORPL_APP_SEQ_LEN  2
#define APP_EXT_TOTAL_LEN  (ORPL_APP_SEQ_LEN)
#endif /*ORPL_BITMAP_DUPLICATE_DETECTION */
#endif

#if UDP_CONF_FRAG
#include "udp-fragment.h"
#endif
#include <string.h>

/*---------------------------------------------------------------------------*/
void
uip_udp_packet_send(struct uip_udp_conn *c, const void *data, int len)
{
#if UIP_UDP
  if(data != NULL) {
#if UDP_CONF_FRAG
    udp_fragment_packet_send(c,data,len);     
#else  
    uip_udp_conn = c;
    char buf[10];
    int pos =0;
#if ORPL_BITMAP_DUPLICATE_DETECTION 
    uint16_t current_seqno=orpl_bitmap_get_current_seqno();
  //  printf("send seqno:%02x\n",current_seqno);
    memcpy(buf+pos,&current_seqno, sizeof(current_seqno));
  //   memcpy(&buf[1],&current_seqno, sizeof(current_seqno));
    pos+=2;
#endif /* ORPL_BITMAP_DUPLICATE_DETECTION */ 
    uip_slen = len + pos;

    memmove(&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN],buf,pos);
    memmove(&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN+pos], data,
            len > UIP_BUFSIZE - UIP_LLH_LEN - UIP_IPUDPH_LEN -pos?
            UIP_BUFSIZE - UIP_LLH_LEN - UIP_IPUDPH_LEN - pos: len);

    uip_process(UIP_UDP_SEND_CONN);

#if UIP_CONF_IPV6_MULTICAST
  /* Let the multicast engine process the datagram before we send it */
  if(uip_is_addr_mcast_routable(&uip_udp_conn->ripaddr)) {
    UIP_MCAST6.out();
  }
#endif /* UIP_IPV6_MULTICAST */

#if NETSTACK_CONF_WITH_IPV6
    tcpip_ipv6_output();
#else
    if(uip_len > 0) {
      tcpip_output();
    }
#endif
#endif /* UDP_CONF_FRAG */
  }
  uip_slen = 0;
#endif /* UIP_UDP */
}
/*---------------------------------------------------------------------------*/
void
uip_udp_packet_sendto(struct uip_udp_conn *c, const void *data, int len,
		      const uip_ipaddr_t *toaddr, uint16_t toport)
{
  uip_ipaddr_t curaddr;
  uint16_t curport;

  if(toaddr != NULL) {
    /* Save current IP addr/port. */
    uip_ipaddr_copy(&curaddr, &c->ripaddr);
    curport = c->rport;

    /* Load new IP addr/port */
    uip_ipaddr_copy(&c->ripaddr, toaddr);
    c->rport = toport;

    uip_udp_packet_send(c, data, len);

    /* Restore old IP addr/port */
    uip_ipaddr_copy(&c->ripaddr, &curaddr);
    c->rport = curport;
  }
}
/*---------------------------------------------------------------------------*/
uint8_t 
uip_udp_received_data_preprocess(void){
  uint16_t offset_len = 0;

#if UDP_CONF_FRAG & UIP_CONF_IPV6_ORPL_BITMAP
  uint8_t frag_type,frag_flag;
#endif

#if ORPL_BITMAP_DUPLICATE_DETECTION & UIP_CONF_IPV6_ORPL_BITMAP
  offset_len += 2; 
#endif

#if UDP_CONF_FRAG & UIP_CONF_IPV6_ORPL_BITMAP
 memcpy(&frag_type, (uint8_t *)(uip_appdata + offset_len), sizeof(frag_type));
 frag_flag = (frag_type>>6)&3;
 if(frag_flag){
//    uip_clear_buf();    //clear frag packet ,drop it
    return 0;
 }else{
    offset_len += 1;
 }
#endif
 
  uip_len = uip_len - offset_len;
  uip_appdata = &uip_buf[UIP_IPUDPH_LEN + UIP_LLH_LEN + offset_len];
  return 1;
}

/*---------------------------------------------------------------------------*/
