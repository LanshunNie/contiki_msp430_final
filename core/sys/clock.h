/*
 * Copyright (c) 2004, Swedish Institute of Computer Science.
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
 */

/** \addtogroup sys
 * @{
 */

/**
 * \defgroup clock Clock library
 *
 * The clock library is the interface between Contiki and the platform
 * specific clock functionality. The clock library defines a macro,
 * CLOCK_SECOND, to convert seconds into the tick resolution of the platform.
 * Typically this is 1-10 milliseconds, e.g. 4*CLOCK_SECOND could be 512.
 * A 16 bit counter would thus overflow every 1-10 minutes.
 * Platforms use the tick interrupt to maintain a long term count
 * of seconds since startup.
 *
 * Platforms may also implement rtimers for greater time resolution
 * and for real-time interrupts, These use a corresponding RTIMER_SECOND.
 *
 * \note These timers do not necessarily have a common divisor or are phase locked.
 * One may be crystal controlled and the other may not. Low power operation
 * or sleep will often use one for wake and disable the other, then give
 * it a tick correction after wakeup.
 *
 * \note The clock library need in many cases not be used
 * directly. Rather, the \ref timer "timer library", \ref etimer
 * "event timers", or \ref rtimer "rtimer library" should be used.
 *
 * \sa \ref timer "Timer library"
 * \sa \ref etimer "Event timers"
 * \sa \ref rtimer "Realtime library"
 *
 * @{
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include "contiki-conf.h"

/**
 * A second, measured in system clock time.
 *
 * \hideinitializer
 */
#ifdef CLOCK_CONF_SECOND
#define CLOCK_SECOND CLOCK_CONF_SECOND
#else
#define CLOCK_SECOND (clock_time_t)32
#endif

/**
 * Initialize the clock library.
 *
 * This function initializes the clock library and should be called
 * from the main() function of the system.
 *
 */
 
#if TRXEB1120_CONF_LOWPOWER
typedef struct {
// 	// uint16_t year;
// 	// uint8_t mon;
// 	// uint8_t day;
// 	// uint8_t day_of_week;
	uint8_t hour;
	uint8_t minute;
	uint8_t sec;
} soft_time;
#endif

void clock_init(void);

/**
 * Get the current clock time.
 *
 * This function returns the current system clock time.
 *
 * \return The current clock time, measured in system ticks.
 */
CCIF clock_time_t clock_time(void);

/**
 * Get the current value of the platform seconds.
 *
 * This could be the number of seconds since startup, or
 * since a standard epoch.
 *
 * \return The value.
 */
CCIF unsigned long clock_seconds(void);

/**
 * Set the value of the platform seconds.
 * \param sec   The value to set.
 *
 */
void clock_set_seconds(unsigned long sec);

/**
 * Wait for a given number of ticks.
 * \param t   How many ticks.
 *
 */
void clock_wait(clock_time_t t);

/**
 * Delay a given number of microseconds.
 * \param dt   How many microseconds to delay.
 *
 * \note Interrupts could increase the delay by a variable amount.
 */
void clock_delay_usec(uint16_t dt);

/**
 * Deprecated platform-specific routines.
 *
 */
int clock_fine_max(void);
unsigned short clock_fine(void);
void clock_delay(unsigned int delay);

#if TRXEB1120_CONF_LOWPOWER
void rtc_init (void);

// RTCYEAR = 0x2016;                         // Year = 0x2010
//   RTCMON = 0x3;                             // Month = 0x04 = April
//   RTCDAY = 0x4;                            // Day = 0x05 = 5th
//   RTCDOW = 0x5;                            // Day of week = 0x01 = Monday
//   RTCHOUR = 0x22;                           // Hour = 0x10
//   RTCMIN = 0x00;                            // Minute = 0x32
//   RTCSEC = 0x00;
// void read_calendar(calendar_time * caltime);
// void write_calendar(calendar_time  caltime);

void set_active_flag(int hour,int minute,int second_s);//get active flag from 

void syn_update_timenow(soft_time syntime);

void get_timenow(soft_time *times);

uint8_t get_active_flag(void);

clock_time_t get_idle_time(void);

void set_init_flag(uint16_t  flag);
     
uint16_t get_init_flag();    

void set_autocal_info(int autocal_offest,uint32_t autocal_interval);
uint16_t get_nowdays();

void set_ledon_flag(uint8_t flag);
//zhangwei set changed for load balance
unsigned long real_time_clock_second(void);
#endif

#endif /* CLOCK_H_ */

/** @} */
/** @} */
