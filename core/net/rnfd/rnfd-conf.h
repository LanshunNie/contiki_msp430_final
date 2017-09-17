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
 *	 RNFD configure.   // root node fail detect
 * \author
 *	@yang
 *
 */

#ifndef RNFD_CONF_H
#define RNFD_CONF_H

#include "contiki.h"

/* node send root suspect message condition, link callback continue many times MAC_TX_NOACK */
#define RNFD_CONTINUE_NOACK_MAXNUM  3 //5  4
/* node send root suspect message max time */
#define RNFD_SEND_SUSPECTMSG_MAXNUM  3 //3
/* node send root suspect message time interval. */
#define RNFD_SEND_SUSPECTMSG_BASETIME  50
#define RNFD_SEND_SUSPECTMSG_RANDOMTIME  20

/* after node send final root suspect message,set send root offline message time interval. */
#define RNFD_SEND_OFFLINEMSG_INTERVAL  (2*60)  //5*60
/* after send root offline message ,free root dodag ,set root dodag enter blacklist,this is set 
   how long root dodag stay in blacklist . */
#define RNFD_DELETE_OFFLINE_INTERVAL   (30 * 60)
/* root online state valid lifetime; if exceed this time,root online state is out of date.*/
//#define RNFD_ONLINE_VALID_LIFETIME     (5 * 60)

#endif /* RNFD_CONF_H */





