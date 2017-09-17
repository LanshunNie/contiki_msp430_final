/*
 * \addtogroup energy-efficiency
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: energy-efficiency.c,v 2.0 2017/02 /21 
 */

/**
 * \file
 *         A simple energy-efficiency mechanism
 * \author
 *         zhangwei
 */

#include "contiki.h"
#include "node-id.h"
#include <stdio.h>
#include <string.h>
#include "lib/random.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "net/mac/energy-efficiency/energy-efficiency.h"
/*---------------------------------------------------------------------------*/
/* cal weight */
#define CAL_CURRENT_WEIGHT1		5
#define CAL_CURRENT_WEIGHT2 	2
#define WAKEUP_RATE_STEP      8
#define WAKEUP_INTERVAL_STEP	RTIMER_ARCH_SECOND / WAKEUP_RATE_STEP
#define BEHAVIOUR_CHANGE_THRESHOLD 3
#define BEHAVIOUR_LOCKED_THRESHOLD 4  

/*setting unit of current as nA for accutacy  */
#define LPM_CURRENT        4256
#define CPU_CURRENT        3280000
#define LISTEN_CURRENT     25000000
#define TRANSMIT_CURRENT   157540000
static uint32_t current;

#define DEDUG 0
/*---------------------------------------------------------------------------*/

static node_behaviour_table_t node_behaviour_table[NODE_BEHAVIOUR_TABLE_SIZE];

static uint8_t behaviour_tb_index = 0;
static uint8_t current_index = 0xFF;
static uint8_t current_type[3];
// static uint64_t total_lpm_time;
// static uint64_t total_cpu_time;
// static uint64_t total_listen_time;
// static uint64_t total_transmit_time;
// static uint16_t duty_cycle_table[5];

energy_efficiency_t energy_efficiency;
/*---------------------------------------------------------------------------*/

static uint16_t ladder_of_cal_next_cycle_time(void);
static uint16_t residual_energy_of_cal_next_cycle_time(void);
static void adjust_energy_efficiency_effect(unsigned long current_dc,uint32_t present_cur);
static void set_eneygy_efficiency_direction(uint16_t old_cycle_time);
static void energy_efficiency_cal_expect_current(uint32_t present_cur);
static void energy_efficiency_draw_behaviour(void);
static uint32_t energy_efficiency_cal_present_current(void);
/*---------------------------------------------------------------------------*/

void
energy_efficiency_init(uint32_t current_budget,uint16_t con_rdc_cycle_time,uint32_t guard_current)
{

  memset(&energy_efficiency, 0, sizeof(energy_efficiency))  ;
  energy_efficiency.current_budget  = current_budget  ;
  energy_efficiency.con_rdc_cycle_time = con_rdc_cycle_time ;
  energy_efficiency.guard_current   = guard_current   ;
  energy_efficiency.expect_current  = current_budget  ;
  energy_efficiency.rdc_cycle_time     = con_rdc_cycle_time ;
  energy_efficiency.energy_efficiency_effect = ENERGY_EFFICIENCY_INVALID;

}

/*---------------------------------------------------------------------------*/

void 
energy_efficiency_cal_rdc_cycle_time(void)
{
  if(behaviour_tb_index >= NODE_BEHAVIOUR_TABLE_SIZE)
  {
    uint32_t present_cur = 0;
    uint16_t old_cycle_time=0;

    present_cur = energy_efficiency_cal_present_current();
    
    energy_efficiency_cal_expect_current(present_cur);

    old_cycle_time = energy_efficiency.rdc_cycle_time;
    
    
      if(present_cur  < (energy_efficiency.current_budget - energy_efficiency.guard_current)){

        if( energy_efficiency.energy_efficiency_effect == ENERGY_EFFICIENCY_INVALID ||                                                           \
                                                          energy_efficiency.cycle_time_direction == CYCLE_TIME_INCREASE){

          // energy_efficiency_cal_expect_current(present_cur);

          energy_efficiency.rdc_cycle_time = MAX( MIN_RDC_CYCLE_TIME,                                                                            \
                                                  MIN(ladder_of_cal_next_cycle_time(),                                                           \
                                                      residual_energy_of_cal_next_cycle_time()                                                   \
                                                     )
                                                );

          // printf("CYCLE_TIME_INCREASE %d \n", energy_efficiency.rdc_cycle_time);
          set_eneygy_efficiency_direction(old_cycle_time);

        }

      }

      if(present_cur  > (energy_efficiency.current_budget + energy_efficiency.guard_current)){

        if( energy_efficiency.energy_efficiency_effect == ENERGY_EFFICIENCY_INVALID ||                                                           \
                                                          energy_efficiency.cycle_time_direction == CYCLE_TIME_DECREASE){

          // energy_efficiency_cal_expect_current(present_cur);
        
          if(energy_efficiency.cycle_time_direction == CYCLE_TIME_DECREASE)
          {
            energy_efficiency.rdc_cycle_time += (WAKEUP_INTERVAL_STEP / 3);
          }

          energy_efficiency.rdc_cycle_time = MIN( MAX_RDC_CYCLE_TIME,                                                                            \
                                                  MAX(ladder_of_cal_next_cycle_time(),                                                           \
                                                      residual_energy_of_cal_next_cycle_time()                                                   \
                                                     )                                                         
                                                );

          set_eneygy_efficiency_direction(old_cycle_time);
          // printf("CYCLE_TIME_DECREASE %d \n", energy_efficiency.rdc_cycle_time);

        }
      }

  }

}

/*---------------------------------------------------------------------------*/

uint8_t
energy_efficiency_get_current_node_behauiour(unsigned long now)
{
  unsigned long diff;
  uint8_t index;
  uint8_t current_type;

  diff = (now - energy_efficiency.begin_time) % 600;

  index = 2 * (diff / 15);

  current_type = ((energy_efficiency.behaviour_bit_map[index/8] >> (6- (index % 8))) & 0x03); 

  return current_type;
}
/*---------------------------------------------------------------------------*/

void 
energy_efficiency_node_behaviour_check(void)
{
  uip_ipaddr_t check_ip;
  uip_ip6addr(&check_ip, 0x0000, 0, 0, 0, 0, 0, 0, 0x0000);

  energy_efficiency_adjust_behaviour(check_ip);
 
  #if DEDUG
    uint8_t i=0;
    // energy_efficiency_adjust_behaviour1(check_ip,0);
  #endif
  
  if(behaviour_tb_index >= NODE_BEHAVIOUR_TABLE_SIZE)
  {
    energy_efficiency_draw_behaviour();

    #if DEDUG
      for(i=0;i<10;i++){
        printf("%02x  ", energy_efficiency.behaviour_bit_map[i]);
      }
      printf("\n");

    #endif
  }

  current_index =0xFF;

  behaviour_tb_index++;
}
/*---------------------------------------------------------------------------*/
void 
energy_efficiency_adjust_behaviour(uip_ipaddr_t addr)
{
  uint8_t type=0,temp_type=0;
  uint8_t index = 0 ;
  uint8_t type_index =0;
  uint8_t max=0;
  unsigned long diff=0;

  if(addr.u8[0] == 0 && addr.u8[15] == 0)
  {
    type = NODE_BEHAVIOUR_TYPE_NOTHING;

  }else if(addr.u8[0]== 0xff){

    if((addr.u8[1] == 0x02 && addr.u8[15] == 0x01) || ((addr.u8[1] & 0x0F) > 0x02))
    {

      type = NODE_BEHAVIOUR_TYPE_BROADCAST;

    }else{

      type = NODE_BEHAVIOUR_TYPE_NETWORK;

    }

  }else{

    type = NODE_BEHAVIOUR_TYPE_UNICAST;

  }

  diff = (real_time_clock_second() - energy_efficiency.begin_time) % 600;

  index = 2 * (diff / 15);

  if(index == current_index)
  {
    current_type[type % 3]++;
  
  }else if(current_index == 0xFF){
  
    current_index = index;

    current_type[type % 3]++;
  
  }else{

    for(type_index=0;type_index<3;type_index++){

      if(current_type[type_index]>max)
      {
        temp_type = type_index;

        max=current_type[type_index];
      }

      current_type[type_index] = 0;
    }

    if(max >= BEHAVIOUR_LOCKED_THRESHOLD)
    {
      if(temp_type == 0)
      {
        temp_type = 0x03;
      }

    }else{
      
      temp_type =0 ;
    
    }

    node_behaviour_table[behaviour_tb_index % NODE_BEHAVIOUR_TABLE_SIZE].behaviour_bit_map[current_index/8] |=                                   \
                                                                                          (temp_type << (6- (current_index % 8)));
      
    // printf("temp_type %d ,current_index %d \n",temp_type,current_index );

    current_index = index;
  }

}

void 
energy_efficiency_adjust_behaviour1(uip_ipaddr_t addr,uint16_t now)
{
  uint8_t type=0,temp_type=0;
  uint8_t index = 0 ;
  uint8_t type_index =0;
  uint8_t max=0;
  unsigned long diff=0;
    
  if(addr.u8[0] == 0)
  {
    temp_type = NODE_BEHAVIOUR_TYPE_NOTHING;

  }else if(addr.u8[0]== 0xff){
    
    if((addr.u8[1] == 0x02 && addr.u8[15] == 0x01) || ((addr.u8[1] & 0x0F) > 0x02))
    {

      type = NODE_BEHAVIOUR_TYPE_BROADCAST;

    }else{

      type = NODE_BEHAVIOUR_TYPE_NETWORK;

    }

  }else{

    type = NODE_BEHAVIOUR_TYPE_UNICAST;

  }
  
  diff = (now - energy_efficiency.begin_time) % 600;

  index = 2 * (diff / 15);

  if(index == current_index)
  {
    current_type[type % 3]++;
  
  }else if(current_index == 0xFF){
  
    current_index = index;

    current_type[type % 3]++;
  
  }else{

    for(type_index=0;type_index<3;type_index++){

      if(current_type[type_index]>max)
      {
        temp_type = type_index;

        max=current_type[type_index];
      }

      current_type[type_index] = 0;
    }

    if(max > BEHAVIOUR_LOCKED_THRESHOLD)
    {
      if(temp_type == 0)
      {
        temp_type = 0x03;
      }

    }else{
      
      temp_type =0 ;
    
    }

    node_behaviour_table[behaviour_tb_index % NODE_BEHAVIOUR_TABLE_SIZE].behaviour_bit_map[current_index/8] |=                                   \
                                                                                          (temp_type << (6- (current_index % 8)));
    
    current_index = index;
  }

}
/*---------------------------------------------------------------------------*/

static uint16_t 
ladder_of_cal_next_cycle_time(void)
{
  rpl_dag_t *dag;
  uint8_t hop = 1;
  uint8_t rdc_check_rate;
  uint16_t cycle_time = energy_efficiency.rdc_cycle_time;
  int16_t interval_step;

  dag = rpl_get_any_dag();

  if(dag != NULL) 
  {
    hop = dag->rank / 256 -1;
  }

  switch(hop){
    case 1:
      rdc_check_rate = 7;
      break;
    case 2:
      rdc_check_rate = 6;
      break;
    case 3:
      rdc_check_rate = 5;
      break;
    case 4:
      rdc_check_rate =4;
      break;
    case 5:
      rdc_check_rate =3;
      break;
    default:
      rdc_check_rate = 2;
      break;
  }

  cycle_time = RTIMER_ARCH_SECOND / rdc_check_rate ;

  interval_step = WAKEUP_INTERVAL_STEP *(int32_t) energy_efficiency.expect_current / energy_efficiency.current_budget                              \
                 - WAKEUP_INTERVAL_STEP;

  if(interval_step > 0)
  {
    cycle_time = MIN(cycle_time + interval_step,MAX_RDC_CYCLE_TIME);
  }                                                   
  
  if(interval_step<0)
  {
    if((-interval_step) > (int16_t)cycle_time)
    {

      interval_step = cycle_time / 4;

      cycle_time = cycle_time - interval_step;

    }else{

      cycle_time = cycle_time + interval_step;

    }

  }

  energy_efficiency.lad_rdc_cycle_time = cycle_time;

	return cycle_time; 
}
/*---------------------------------------------------------------------------*/

static uint16_t 
residual_energy_of_cal_next_cycle_time(void)
{
 
  uint16_t cycle_time = energy_efficiency.rdc_cycle_time;
  int16_t interval_step = 0;
  
  interval_step = WAKEUP_INTERVAL_STEP *(int32_t) energy_efficiency.expect_current / energy_efficiency.current_budget                              \
                 - WAKEUP_INTERVAL_STEP;

  // printf("residual_energy interval_step %d\n",interval_step );
  if(interval_step > 0)
  {

    interval_step = MIN(WAKEUP_INTERVAL_STEP,interval_step);

    cycle_time = MIN(energy_efficiency.rdc_cycle_time + interval_step,MAX_RDC_CYCLE_TIME);
  
  }                                                   
  
  if(interval_step<0)
  {
    if((-interval_step) > (int16_t)(energy_efficiency.rdc_cycle_time))
    {

      interval_step = energy_efficiency.rdc_cycle_time / 4;

    }else{
      
      interval_step = MIN((-interval_step) , WAKEUP_INTERVAL_STEP); 
    
    }

    cycle_time = energy_efficiency.rdc_cycle_time - interval_step;
  }
  // printf("residual_energy %d \n", cycle_time);

  energy_efficiency.red_rdc_cycle_time = cycle_time;

	return cycle_time;

}

/*---------------------------------------------------------------------------*/
static void  
adjust_energy_efficiency_effect(unsigned long current_dc,uint32_t present_cur)
{

  if( energy_efficiency.prev_duty_cycle != 0 && energy_efficiency.cycle_time_direction == CYCLE_TIME_INCREASE){
    
    if(energy_efficiency.prev_duty_cycle < current_dc){//energy efficiency effect

      energy_efficiency.energy_efficiency_effect = ENERGY_EFFICIENCY_EFFECT;

    }else{

      energy_efficiency.energy_efficiency_effect = ENERGY_EFFICIENCY_INVALID;

    }

  }

  if( energy_efficiency.prev_duty_cycle != 0 && energy_efficiency.cycle_time_direction == CYCLE_TIME_DECREASE){
    
    if(energy_efficiency.prev_duty_cycle > current_dc){//energy efficiency effect

      energy_efficiency.energy_efficiency_effect = ENERGY_EFFICIENCY_EFFECT;

    }else{

      energy_efficiency.energy_efficiency_effect = ENERGY_EFFICIENCY_INVALID;

    }

  }

  if( energy_efficiency.prev_duty_cycle != 0 && energy_efficiency.cycle_time_direction == CYCLE_TIME_MAINTAIN){
       
    if(present_cur  < (energy_efficiency.current_budget + energy_efficiency.guard_current) &&                                                    \
                                              present_cur  > (energy_efficiency.current_budget - energy_efficiency.guard_current)){//energy efficiency effect

      energy_efficiency.energy_efficiency_effect = ENERGY_EFFICIENCY_EFFECT;

    }else{

      energy_efficiency.energy_efficiency_effect = ENERGY_EFFICIENCY_INVALID;

    }

  }

}
/*---------------------------------------------------------------------------*/
static void 
set_eneygy_efficiency_direction(uint16_t old_cycle_time)
{
  if(old_cycle_time > energy_efficiency.rdc_cycle_time){

    energy_efficiency.cycle_time_direction = CYCLE_TIME_DECREASE;

  }else if(old_cycle_time < energy_efficiency.rdc_cycle_time){

    energy_efficiency.cycle_time_direction = CYCLE_TIME_INCREASE;

  }else{

    energy_efficiency.cycle_time_direction = CYCLE_TIME_MAINTAIN;

  }

}
/*---------------------------------------------------------------------------*/

static void 
energy_efficiency_cal_expect_current(uint32_t present_cur)
{
  uint32_t expect_current = 0,current_bg = 0,guard_current = 0;

  current_bg = energy_efficiency.current_budget;

  expect_current = energy_efficiency.expect_current;

  guard_current = energy_efficiency.guard_current;

  if((present_cur - expect_current < 50 || expect_current - present_cur <50) &&                                                                    \
     (current_bg - expect_current > guard_current || expect_current - current_bg > guard_current)                                                \
    )
  {
    expect_current = energy_efficiency.current_budget;
  }

  expect_current = (present_cur * CAL_CURRENT_WEIGHT2 + expect_current * (CAL_CURRENT_WEIGHT1 - CAL_CURRENT_WEIGHT2))                            \
                    / CAL_CURRENT_WEIGHT1 ;
  // printf("expect_cur %d\n",expect_cur );
  energy_efficiency.expect_current = expect_current;
}
/*---------------------------------------------------------------------------*/

static uint32_t 
energy_efficiency_cal_present_current(void){

  uint64_t transmit =0,listen,lpm=0,cpu=0;
  uint16_t present_dc=0;
  uint32_t present_cur=0;

  energest_flush();

  cpu      = energest_type_time(ENERGEST_TYPE_CPU)      ;
  lpm      = energest_type_time(ENERGEST_TYPE_LPM)      ;
  transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT) ;
  listen   = energest_type_time(ENERGEST_TYPE_LISTEN)   ;

  present_cur =  ( listen * LISTEN_CURRENT + transmit * TRANSMIT_CURRENT + lpm * LPM_CURRENT + cpu * CPU_CURRENT) / ( lpm+ cpu ) ;

  present_dc =  ( lpm + cpu ) / ( listen + transmit) ;

  adjust_energy_efficiency_effect(present_dc,present_cur);

  energy_efficiency.prev_duty_cycle = present_dc;

  current = present_cur;

  return present_cur;

}
/*---------------------------------------------------------------------------*/

static void
energy_efficiency_draw_behaviour(void)
{
  uint8_t i=0,j=0,index=0;
  uint8_t unicast_count=0,broadcasr_count=0,nothing_count=0,network_count=0;
  uint8_t type=0;

  memset(&energy_efficiency.behaviour_bit_map, 0, sizeof(energy_efficiency.behaviour_bit_map));

  for(;index<40;index++)
  {
    j = index * 2;

    for(i=0;i<NODE_BEHAVIOUR_TABLE_SIZE;i++)
    {
      type = (node_behaviour_table[i].behaviour_bit_map[j/8] >> (6 - j%8)) & 0x03;

      // printf("type %d ,index %d\n",type,j/8 );

      if( type == NODE_BEHAVIOUR_TYPE_UNICAST)
      {
        if((++unicast_count) > BEHAVIOUR_CHANGE_THRESHOLD)
        {
          energy_efficiency.behaviour_bit_map[j/8] |= (type << (6- (j % 8)));
          break;
        }
      }
      else if(type == NODE_BEHAVIOUR_TYPE_BROADCAST)
      {
        if((++broadcasr_count) > BEHAVIOUR_CHANGE_THRESHOLD)
        {
          energy_efficiency.behaviour_bit_map[j/8] |= (type << (6- (j % 8)));
          break;
        }
      }
      else if(type == NODE_BEHAVIOUR_TYPE_NETWORK)
      {
        if((++network_count) > BEHAVIOUR_CHANGE_THRESHOLD)
        {
          energy_efficiency.behaviour_bit_map[j/8] |= (type << (6- (j % 8)));
          break;
        }
      }
      else if(type == NODE_BEHAVIOUR_TYPE_NOTHING)
      {
        if((++nothing_count) > BEHAVIOUR_CHANGE_THRESHOLD)
        {
          energy_efficiency.behaviour_bit_map[j/8] |= (type << (6- (j % 8)));
          break;
        }
      }
    }
  } 
}
/*---------------------------------------------------------------------------*/

uint32_t getCurrent(void){
  return current;
}
/* energy efficiency .c */