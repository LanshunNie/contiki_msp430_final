/**
 * \addtogroup task-schedule
 * @{
 */


/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: task-schedule.c,v 1.21 2016/3/31 
 */

/**
 * \file
 *         A simple task-schedule mechanism
 * \author
 *         zhangwei
 */

#ifndef __TASK_SCHEDULE_H__
#define __TASK_SCHEDULE_H__

#include "sys/etimer.h"

#define MUST_TASK    11
#define TEMP_TASK    12

#define TASK_READY  22
#define TASK_RUNNING   23
#define TASK_YEILD   24

#define TASK_COUNT_DEFAULT 0
#define TASK_PERIOD_DEFAULT 1
struct task_schedule {
  struct task_schedule *next;
  struct etimer etimer;
  struct process *p;
  void (*f)(void *);
  void *ptr;
  int task_type;
  int task_state;
  int task_period;
  int task_exc_count;
};

void task_schedule_set(struct task_schedule *c,int ts_type,int ts_state,int task_period,
		void (*f)(void *), void *ptr);

void task_schedule_init(void);

void task_schedule_change(void);

void task_schedule_stop(struct task_schedule *c);

void task_schedule_set_period(struct task_schedule *t,int ts_period);

#endif 
/* __TASK_SCHEDULE_H__ */
/** @} */
/** @} */
