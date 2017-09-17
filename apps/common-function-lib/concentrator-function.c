/*
 * Copyright (c) 2016, yang.
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
 * This file is concentrator deal function.
 * 
 * Author:@yang
 *
 */

#include "contiki.h"
#include "concentrator-function.h"
#include "net/ip/uip.h"
#include "net/linkaddr.h"
#include "net/netstack.h"

#include <stdio.h>
#include <string.h>

static uip_ipaddr_t concentrator_ipv6addr;

/*-------------------------------------------------------------------------*/
void 
init_concentrator_ipv6addr(void){
   uip_ip6addr(&concentrator_ipv6addr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0x0002);
}
/*-------------------------------------------------------------------------*/
uip_ipaddr_t* 
get_concentrator_ipv6addr(void){
  return &concentrator_ipv6addr;
}
/*-------------------------------------------------------------------------*/
void 
set_concentrator_ipv6addr(const uip_ipaddr_t *ip6addr){
   uip_ip6addr_copy(&concentrator_ipv6addr,ip6addr); 
}
/*-------------------------------------------------------------------------*/




