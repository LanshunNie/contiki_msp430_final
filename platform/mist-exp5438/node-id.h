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
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: node-id.h,v 1.1 2010/08/24 16:26:38 joxe Exp $
 */

#ifndef __NODE_ID_H__
#define __NODE_ID_H__
#include <inttypes.h>

#define NORMAL   0x01
#define ABNORMAL 0x00

#define LENGTH_OF_METER_READ_CMD  30

void node_id_restore(void);
void node_id_burn(unsigned short node_id);

extern unsigned short node_id;
extern unsigned char node_mac[8];

uint8_t channel_byte;  

uint16_t restart_count;  
uint16_t normal_byte;
uint8_t  cmd_read_meter[LENGTH_OF_METER_READ_CMD];
// uint64_t time_cpu;
// uint64_t time_lpm;
// uint64_t time_transmit;
// uint64_t time_listen;

void normalbyte_rfchannel_burn(uint16_t normalbyte , uint8_t rfchannel);
void normalbyte_rfchannel_restore(void);

void restart_count_byte_burn(unsigned short val);
void restart_count_byte_restore(void);

int cmd_bytes_burn(uint8_t *array);
void restore_meter_cmd(void);
void print_cmd_array(void);

// void energy_bytes_init_burn(void);
// void store_energy(uint64_t cpu,uint64_t lpm,uint64_t trans,uint64_t listen);
// void get_energy(uint64_t *cpu,uint64_t *lpm,uint64_t *trans,uint64_t *listen);
// void energy_bytes_restore();

#endif /* __NODE_ID_H__ */
