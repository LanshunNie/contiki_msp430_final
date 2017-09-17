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
#include "task-schedule.h"
#include "contiki.h"
#include "lib/list.h"
#include "net/netstack.h"
#include "node-id.h"
#include "net/rpl/rpl.h"
#include "netsynch.h"
#include "contiki-net.h"

#include "lib/random.h"//add by gxn

#include <stdio.h>

#define HASH_SCHEDULE 247

#define time_interval() NETSTACK_RDC.channel_check_interval()
#define TIME_SLOAT   250 * time_interval()

LIST(task_schedule_list);

static char initialized;
static struct ctimer reset_timer;


static int calculate_hash(void);
static clock_time_t get_Start_Time(void);
static void task_schedule_reset(void *tttt); //chang by gxn
static uint16_t get_rank(void);
static clock_time_t guard_time = 8 * CLOCK_SECOND;
#if ROOTNODE
static clock_time_t last_idel_time =0;
#endif
/*---------------------------------------------------------------------------*/
PROCESS(task_schedule_process, "task_schedule process");
PROCESS_THREAD(task_schedule_process, ev, data)
{
  struct task_schedule *c;
  PROCESS_BEGIN();

  initialized = 1;
  
  while(1) {

    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_TIMER);

    for(c = list_head(task_schedule_list); c != NULL; c = c->next) {

      if(&c->etimer == data && c->task_state == TASK_READY) 
      {

        PROCESS_CONTEXT_BEGIN(c->p);
        if(c->f != NULL) 
        {
          c->f(c->ptr);
        }
        PROCESS_CONTEXT_END(c->p);

      	if(c->task_type == MUST_TASK)
        {

      	  if(c->task_state == TASK_READY)
          {

      	  	c->task_state = TASK_RUNNING;

      	  }

        }
        else if(c->task_type == TEMP_TASK)
        {
            
            list_remove(task_schedule_list,c);
            
        }

      }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
task_schedule_init(void)
{
  initialized = 0;

  list_init(task_schedule_list);

  process_start(&task_schedule_process, NULL); 
}
/*---------------------------------------------------------------------------*/
static int calculate_hash(void){
  
  return( UIP_HTONS(node_id) % HASH_SCHEDULE );

}
/*---------------------------------------------------------------------------*/

static clock_time_t get_Start_Time(void){

    if(initialized) {

      struct task_schedule *t;
      int start_index = 0;
      clock_time_t start_time;

      for(t = list_head(task_schedule_list); t != NULL; t = t->next) {
      
          if(t->task_state == TASK_READY){
              start_index++;
          } 
        }  
        
#if ROOTNODE
      // if(get_idle_time() * CLOCK_SECOND >7.5 * 60 *CLOCK_SECOND)
        if(last_idel_time == 0)
        {

          start_index += 3;

        }else if( last_idel_time * CLOCK_SECOND - get_idle_time() * CLOCK_SECOND < 60 * CLOCK_SECOND){

          start_index += 1;

        }
#endif
      // printf("start_index = %d\n",start_index );
      //start_time = (TIME_SLOAT / 2) + 2 * start_index * TIME_SLOAT + calculate_hash()*time_interval() * 1.5 + (random_rand() % ( 2 * time_interval()) ) * get_rank();
            
      start_time = guard_time + start_index * TIME_SLOAT * 1.7 + calculate_hash()*time_interval() * 2 + (random_rand() % ( 2 * time_interval()) ) * get_rank();

#if ROOTNODE

      last_idel_time = get_idle_time();

#endif
      // return 5*CLOCK_SECOND;


      if(get_idle_time()*CLOCK_SECOND<=TIME_SLOAT/2){

        return start_time + get_idle_time() *CLOCK_SECOND - start_index * TIME_SLOAT;
      }

      if(start_time <= (get_idle_time() *CLOCK_SECOND - TIME_SLOAT/2)){
        return start_time;
      }
      else 
      {
        return start_time + get_idle_time() *CLOCK_SECOND - start_index * TIME_SLOAT;
      }

    }
    
    return 0;

}
/*---------------------------------------------------------------------------*/

void
task_schedule_set(struct task_schedule *c,int ts_type,int ts_state,int ts_period,
	   void (*f)(void *), void *ptr)
{
  c->p = PROCESS_CURRENT();
  c->f = f;
  c->ptr = ptr;
  c->task_type = ts_type;
  c->task_period = ts_period;
  c->task_exc_count = 0;
    if(initialized) {

      PROCESS_CONTEXT_BEGIN(&task_schedule_process);

        if(netsynch_is_synched()){
          
          etimer_set(&c->etimer, get_Start_Time());
          c->task_state = ts_state;
        }else{
          c->task_state = TASK_YEILD;
        }

      PROCESS_CONTEXT_END(&task_schedule_process);

    } else {

      c->etimer.timer.interval = get_Start_Time();
      c->task_state = ts_state;
  
    }

  list_add(task_schedule_list, c);
  if(!netsynch_is_synched()){

    task_schedule_reset(NULL);
  }
}


/*---------------------------------------------------------------------------*/
void
task_schedule_change(void)
{
   if(initialized) {

    struct task_schedule *t;
    
    guard_time = TIME_SLOAT /2;

#if ROOTNODE
      last_idel_time = 0;
#endif

    PROCESS_CONTEXT_BEGIN(&task_schedule_process);

     for(t = list_head(task_schedule_list); t != NULL; t = t->next) {
      if(t->task_type == MUST_TASK){

        if( (t->task_state == TASK_RUNNING) && (++(t->task_exc_count) % t->task_period ==0)){

          etimer_set(&t->etimer,get_Start_Time());

          t->task_state = TASK_READY;

          
        } 
      }  
    }

    guard_time= ( UIP_HTONS(node_id) % 37 )*time_interval() + random_rand()%(2*CLOCK_SECOND);

#if ROOTNODE
      last_idel_time = get_idle_time();
#endif

    PROCESS_CONTEXT_END(&task_schedule_process);
  }
}  

/*---------------------------------------------------------------------------*/
static void task_schedule_reset(void*tttt)
{
  
  if(initialized) {

    struct task_schedule *t;

    if(!netsynch_is_synched()){

      ctimer_set(&reset_timer,CLOCK_SECOND*2,task_schedule_reset,NULL);
      return;
    }

    PROCESS_CONTEXT_BEGIN(&task_schedule_process);

    for(t = list_head(task_schedule_list); t != NULL; t = t->next) {
        if(t->task_state == TASK_YEILD){

          etimer_set(&t->etimer,get_Start_Time());
          t->task_state = TASK_READY;

        } 
      }  
    
    PROCESS_CONTEXT_END(&task_schedule_process);

    ctimer_stop(&reset_timer);
  }
}

/*---------------------------------------------------------------------------*/

void task_schedule_set_period(struct task_schedule *ts,int ts_period)
{

  if(initialized) {

    // PROCESS_CONTEXT_BEGIN(&task_schedule_process);

    ts->task_period=ts_period;
    ts->task_exc_count=0;

    // PROCESS_CONTEXT_END(&task_schedule_process);

  }

  // list_add(task_schedule_list, ts);

}
/*---------------------------------------------------------------------------*/

void task_schedule_stop(struct task_schedule *c)
{
  if(initialized) {
    etimer_stop(&c->etimer);
  } else {
    c->etimer.next = NULL;
    c->etimer.p = PROCESS_NONE;
  }
  list_remove(task_schedule_list, c);
}
/*---------------------------------------------------------------------------*/


static uint16_t get_rank(void){
    uint16_t rank;
    rpl_dag_t *dag;
    rank = 0;

    dag = rpl_get_any_dag();
    if(dag != NULL) {
      rank =  dag->rank ;
   }
   return rank  / 256;
}

  //       // return (calculate_hash()*CLOCK_SECOND / (time_interval()+1)  + (random_rand() % (getETX()*CLOCK_SECOND / time_interval()))) % (CLOCK_SECOND * TIME_SLOAT);
  //       // return node_id*CLOCK_SECOND / (time_interval()+1) + (random_rand() % (getETX()*CLOCK_SECOND / time_interval()));
  //       return calculate_hash()*time_interval() + (random_rand() % (time_interval()));
/*---------------------------------------------------------------------------*/
