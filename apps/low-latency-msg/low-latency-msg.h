/**
 * \addtogroup orpl Low latency msg control for root transmited mechanism process
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: low-latency-msg.h,v 1.21 2017/04/10 10:20:00
 */

/**
 * \file
 *        A simple Low latency msg control for root mechanism
 * \author
 *         zhangwei
 */


#ifndef __LOW_LATENCY_MSG_H__
#define __LOW_LATENCY_MSG_H__

void low_latency_msg_control_init(void);

void low_latency_msg_send_register(void (*f)(void *));

void low_latency_msg_send(void);

#endif /* __LOW_LATENCY_MSG_H__ */

