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
 * $Id: netsync-auto-calibrate.h,v 1.4 2016/12/05 13:46:15 
 */

/**
 * \file
 *         Header file for net time sync auto calibrate.
 * \author
 *         @yang
 */

#ifndef __NETSYNC_AUTO_CALIBRATE_H__
#define __NETSYNC_AUTO_CALIBRATE_H__

#include "sys/clock.h"
#include "contiki.h"
#include "net/ip/uip.h"

struct netsync_cal_s {
  uint8_t init_flag;
  int cal_offest ;            // 1 second.
  uint32_t autocal_interval;    // unit second. how long to cal one second.
  uint16_t days;               // 
  soft_time last_caltime;   // last calibration time.
};


struct netsync_cal_s * get_autocal_info();
void netsync_autocal_update(int netsyc_offset);
void netsync_cal_time_record(void);
void netsync_autocal_init(void);

#endif /* __NETSYNC_AUTO_CALIBRATE_H__*/


