/**
 * \addtogroup orpl Low latency mechanism process
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: low-latency.h,v 1.21 2017/03/31 20:00:00
 */

/**
 * \file
 *        A simple Low latency mechanism
 * \author
 *         zhangwei
 */


#ifndef __LOW_LATENCY_H__
#define __LOW_LATENCY_H__
#include "clock.h"

uint8_t set_lowLatency_flag(soft_time  timenow);

uint8_t get_lowLatency_flag(void);

void set_low_latency_interval(uint8_t _interval);

uint8_t get_low_latency_interval(void);

void set_low_latency_active_second(uint8_t _second);

uint8_t get_low_latency_active_second(void);

void set_low_latency_channel_clear_flag(uint8_t _packet_seen);

int8_t get_low_latency_active_time(void);

#endif /* __LOW_LATENCY_H__ */

