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
 *	Public configuration and API declarations for ContikiRPL.
 * \author
 *	Joakim Eriksson <joakime@sics.se> & Nicolas Tsiftes <nvt@sics.se>
 *
 */

#include "rpl-conf.h"
#include "contiki-conf.h"
#include "rpl.h"
#if RPL_CONF_CHANGE_DIO_INTERVAL
/*
 * The DIO interval (n) represents 2^n ms.
 *
 * According to the specification, the default value is 3 which
 * means 8 milliseconds. That is far too low when using duty cycling
 * with wake-up intervals that are typically hundreds of milliseconds.
 * ContikiRPL thus sets the default to 2^12 ms = 4.096 s.
 */
#ifdef RPL_CONF_DIO_INTERVAL_MIN
static uint8_t RPL_DIO_INTERVAL_MIN = RPL_CONF_DIO_INTERVAL_MIN;
#else
static uint8_t RPL_DIO_INTERVAL_MIN = 12;
#endif

/*
 * Maximum amount of timer doublings.
 *
 * The maximum interval will by default be 2^(12+8) ms = 1048.576 s.
 * RFC 6550 suggests a default value of 20, which of course would be
 * unsuitable when we start with a minimum interval of 2^12.
 */
#ifdef RPL_CONF_DIO_INTERVAL_DOUBLINGS
static uint8_t RPL_DIO_INTERVAL_DOUBLINGS = RPL_CONF_DIO_INTERVAL_DOUBLINGS;
#else
static uint8_t RPL_DIO_INTERVAL_DOUBLINGS = 8;
#endif

uint8_t get_rpl_dio_interval_min(void){
	return RPL_DIO_INTERVAL_MIN;
}
uint8_t get_rpl_dio_interval_doublings(void){
   return RPL_DIO_INTERVAL_DOUBLINGS;
}

void set_rpl_dio_interval_min_doublings(uint8_t rpl_dio_interval_min,uint8_t rpl_dio_interval_doublings){
	
	rpl_dag_t *dag;
	rpl_instance_t *instance;
	dag=rpl_get_any_dag();
	if(dag==NULL){
		return;
	}

	instance=dag->instance;

	if(rpl_dio_interval_min!=0 && rpl_dio_interval_min!=0xFF)
		RPL_DIO_INTERVAL_MIN = rpl_dio_interval_min;

	if(rpl_dio_interval_doublings!=0 && rpl_dio_interval_doublings!=0xFF )
		RPL_DIO_INTERVAL_DOUBLINGS = rpl_dio_interval_doublings;
 	
	instance->dio_intdoubl = RPL_DIO_INTERVAL_DOUBLINGS;
	instance->dio_intmin = RPL_DIO_INTERVAL_MIN;

}


#endif

