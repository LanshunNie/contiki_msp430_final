/**
 * \addtogroup energy-efficiency
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: energy-efficiency.h,v 2.0 2017/02 /21 
 */

/**
 * \file
 *         A simple energy-efficiency mechanism
 * \author
 *         zhangwei
 */

#ifndef ENERGY_EFFICIENCY_H_
#define ENERGY_EFFICIENCY_H_

#include "contiki-conf.h"
#include <stdio.h>
#include <string.h>
#include "lib/random.h"
#include "net/ip/uip.h"

/*---------------------------------------------------------------------------*/
#define MAX_RDC_CHECK_RATE 16
#define MIN_RDC_CHECK_RATE 2
#define MIN_RDC_CYCLE_TIME RTIMER_ARCH_SECOND / MAX_RDC_CHECK_RATE
#define	MAX_RDC_CYCLE_TIME RTIMER_ARCH_SECOND / MIN_RDC_CHECK_RATE

#define CYCLE_TIME_INCREASE 1
#define CYCLE_TIME_DECREASE 2
#define CYCLE_TIME_MAINTAIN 0

#define ENERGY_EFFICIENCY_EFFECT 11
#define ENERGY_EFFICIENCY_INVALID 22

#define NODE_BEHAVIOUR_TYPE_UNICAST    3
#define NODE_BEHAVIOUR_TYPE_BROADCAST   2
#define NODE_BEHAVIOUR_TYPE_NETWORK  1
#define NODE_BEHAVIOUR_TYPE_NOTHING    0

#define NODE_BEHAVIOUR_TABLE_SIZE  1


/*---------------------------------------------------------------------------*/
/**
 * \brief A data structure used to maintain rdc stats
 *
 */
typedef struct energy_efficiency {

  /** rdc cycle time */
  uint16_t rdc_cycle_time;

  /** node previous duty cycle */
  uint16_t prev_duty_cycle;

  /** node current threshold */
  /*setting unit of current as nA for accutacy*/
  // uint16_t duty_cycle_budget;
  uint32_t current_budget;

  /** node guard current */
  /*setting unit of current as nA for accutacy*/

  // uint16_t guard_duty_cycle;
  uint32_t guard_current;

  /** node expect current */
  /*setting unit of current as nA for accutacy*/
  // uint16_t expect_duty_cycle;
  uint32_t expect_current;

  /** constant rdc cycle time*/
  uint16_t con_rdc_cycle_time;

  /** rdc cycle time by ladder*/
  uint16_t lad_rdc_cycle_time;

  /** rdc cycle time by red_energy*/
  uint16_t red_rdc_cycle_time;

  /** cycle time direction*/
  uint8_t cycle_time_direction;

  /** energy efficiency effect or not*/
  uint8_t energy_efficiency_effect;

  /** node active period start time*/
  unsigned long begin_time;

  /** node behaviour bit map 15seconds per 2bits */
  /** 00 nothing ;11 forward ;01 downward; 10 dio*/
  uint8_t behaviour_bit_map[10];

} energy_efficiency_t;

/*---------------------------------------------------------------------------*/
/**
 * \brief A data structure used to maintain node behaviour stats
 *
 */

typedef struct node_behaviour_table
{
  /** node behaviour bit map 15seconds per 2bits */
  /** 00 nothing ;11 forward ;01 downward; 10 dio*/
  uint8_t behaviour_bit_map[10];

}node_behaviour_table_t;


/*---------------------------------------------------------------------------*/
/* Access macros */
/*---------------------------------------------------------------------------*/

#if WITH_ENERGY_EFFICIENCY
/* Don't access this variable directly, use the macros below */
extern energy_efficiency_t energy_efficiency;

#define ENERGY_EFFICIENCY_GET(x) energy_efficiency.x
#define ENERGY_EFFICIENCY_INIT(x,y,z) energy_efficiency_init(x,y,z)
#define ENERGY_EFFICIENCY_RESET(x) energy_efficiency.x=0
#define ENERGY_EFFICIENCY_SET_X_EQ_Y(x,y) energy_efficiency.x=y
#define ENERGY_EFFICIENCY_ADJUST_BEHAVIOUR(x) energy_efficiency_adjust_behaviour(x)
#else
#define ENERGY_EFFICIENCY_SET_X_EQ_Y(x)
#define ENERGY_EFFICIENCY_GET(x) 0
#define ENERGY_EFFICIENCY_INIT(x,y,z)
#define ENERGY_EFFICIENCY_RESET(x)
#define ENERGY_EFFICIENCY_DRAW_BEHAVIOUR(x) 

#endif /* WITH_ENERGY_EFFICIENCY */
/*---------------------------------------------------------------------------*/
/*setting unit of current as nA for accutacy*/
void energy_efficiency_init(uint32_t current_budget,uint16_t con_rdc_cycle_time,uint32_t guard_current);

void energy_efficiency_cal_rdc_cycle_time(void);

uint8_t energy_efficiency_get_current_node_behauiour(unsigned long now);

void energy_efficiency_adjust_behaviour(uip_ipaddr_t destipaddr);

void energy_efficiency_node_behaviour_check(void);

void energy_efficiency_adjust_behaviour1(uip_ipaddr_t addr,uint16_t now);


// uint64_t get_total_lpm_time(void);
// uint64_t get_total_cpu_time(void);
// uint64_t get_total_listen_time(void);
// uint64_t get_total_transmit_time(void);

/*---------------------------------------------------------------------------*/

#endif /* ENERGY_EFFICIENCY_H_ */
