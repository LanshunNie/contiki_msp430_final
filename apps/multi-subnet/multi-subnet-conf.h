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
 * $Id: multi-subnet-conf.h,v 1.4 2017/03/26 21:29:10 
 */

/**
 * \file
 *         Header file for multi subnet conf.
 * \author
 *         @yang
 */

#ifndef __MULTI_SUBNET_CONF_H__
#define __MULTI_SUBNET_CONF_H__

#undef RPL_CONF_MAX_DAG_PER_INSTANCE
#ifndef RPL_CONF_MAX_DAG_PER_INSTANCE
#define RPL_CONF_MAX_DAG_PER_INSTANCE 1
#endif

#if ROOTNODE 
#define DEFAULT_SUBNET_PANID       0xAAAA
#endif 

#ifndef DEFAULT_SUBNET_PANID
#define DEFAULT_SUBNET_PANID       0xFFFF
#endif

#ifndef SUBNET_PANID_CONF_LIMIT
#define SUBNET_PANID_CONF_LIMIT    1   //1
#endif

#ifndef SUBNET_PANID_CONF_TIMESLOT_DIFF
#define SUBNET_PANID_CONF_TIMESLOT_DIFF   1
#endif

#endif /* __MULTI_SUBNET_CONF_H__*/


