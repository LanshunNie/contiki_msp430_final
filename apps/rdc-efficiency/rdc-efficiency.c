/**
 * \addtogroup orpl energy efficiency process
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: rdc-efficiency.c,v 1.21 2016/12/19 16:47:38 
 */

/**
 * \file
 *         A simple rdc efficiency mechanism
 * \author
 *         zhangwei
 */

#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include "rdc-efficiency.h"
#include "net/mac/contikimac/contikimac.h"
#include "net/ip/uip-udp-packet.h"
#include <stdlib.h>
#include "rnfd.h"
 
/*---------------------------------------------------------------------------*/
PROCESS(rdc_efficiency_process, "rdc efficiency process");
/*---------------------------------------------------------------------------*/
#define CAL 0
#if CAL
#define LPM_CURRENT        4256
#define CPU_CURRENT        3280000
#define LISTEN_CURRENT     25000000
#define TRANSMIT_CURRENT   157540000
#endif
/*---------------------------------------------------------------------------*/
#define PLAN_A 1
#define PLAN_B 2
#define PLAN_C 3

#define MCAST_RDC_UDP_PORT 3130
/*---------------------------------------------------------------------------*/
process_event_t energy_efficient_begin_event;
/*---------------------------------------------------------------------------*/
static uint8_t plan = 3;
static uint8_t energy_efficient_flag = 1;
static struct uip_udp_conn *rdc_control_conn;
/*---------------------------------------------------------------------------*/
static void tcpip_handler(void);
static void char2uint32(char* appdata,uint32_t* data_32,int8_t begin,int8_t end);
/*---------------------------------------------------------------------------*/

static void 
char2uint32(char* appdata,uint32_t* data_32,int8_t begin,int8_t end){
  uint8_t i=0;
  if(end <= 0 || begin <= 0 || end-begin != 3){
    return;
  }

  for(;i<4;i++){

    *data_32 += (((unsigned char)appdata[begin+i]&0xFFFFFFFF)<<(24-8*i));
  }

}

#if CAL
 void cal(void){
   
  uint64_t  lpm=3234122069;
  uint64_t cpu =5263193;
  uint64_t transmit=378358;
  uint64_t  listen=20459374;
  uint32_t current = ( listen * LISTEN_CURRENT + transmit * TRANSMIT_CURRENT + lpm * LPM_CURRENT + cpu * CPU_CURRENT) / ( lpm+ cpu ) ;
  printf("avg current %lu\n", current);
}
#endif
/*---------------------------------------------------------------------------*/
/*
* appdata 
* 0 flag
* 1-4 budget high->low
* 5-8 guard  high->low  
*/
static void 
tcpip_handler(void)
{
  // leds_toggle(LEDS_ALL);
  if(uip_newdata()) 
  {


    unsigned char appdata_length = 0;
    uint8_t flag = 0;
    uint32_t budget=0,guard=0;

    uip_udp_received_data_preprocess();

    appdata_length = (unsigned char)((char *)uip_appdata)[0];

    // char *appdata = (char *)malloc((appdata_length +1) *sizeof(char));
    char appdata[appdata_length+1];

    memcpy(appdata,(char *)uip_appdata + 1,sizeof(appdata));

    appdata[appdata_length] = '\0';


    flag = appdata[0];

    energy_efficient_flag = flag;

    char2uint32(appdata,&budget,1,4);
    char2uint32(appdata,&guard,5,8);

   
    ENERGY_EFFICIENCY_SET_X_EQ_Y(current_budget,budget);
    ENERGY_EFFICIENCY_SET_X_EQ_Y(guard_current,guard);
    ENERGY_EFFICIENCY_SET_X_EQ_Y(expect_current,budget);
    reset_behaviour_tb_index();
    // printf("budget %lu\n",budget );

    // printf("guard %lu\n",guard );
    // free(appdata);
  }
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(rdc_efficiency_process, ev, data)
{

  PROCESS_BEGIN(); 
  PROCESS_PAUSE();

  energy_efficient_begin_event= process_alloc_event();
  

  rdc_control_conn = udp_new(NULL, UIP_HTONS(0), NULL);

  udp_bind(rdc_control_conn, UIP_HTONS(MCAST_RDC_UDP_PORT));


  while(1) {

    PROCESS_YIELD(); 

    if(ev == energy_efficient_begin_event)
    {
      energy_efficiency_node_behaviour_check();

      if(energy_efficient_flag == 1)
      {
        energy_efficiency_cal_rdc_cycle_time();
      }
      
    }else if(ev == tcpip_event) {
      tcpip_handler();
    }else if(energy_efficient_flag == 1 && get_rnfd_offline_nodag() == 0){

    #if CAL
      cal();
    #endif  

      if(plan == PLAN_A){

        if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) != NODE_BEHAVIOUR_TYPE_UNICAST){

          set_cycletime(ENERGY_EFFICIENCY_GET(con_rdc_cycle_time));

        }

        if(ENERGY_EFFICIENCY_GET(cycle_time_direction) == CYCLE_TIME_INCREASE)
        {

          if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) == NODE_BEHAVIOUR_TYPE_UNICAST)
          {

            set_cycletime(ENERGY_EFFICIENCY_GET(rdc_cycle_time));
        
          }

        }

        if(ENERGY_EFFICIENCY_GET(cycle_time_direction) == CYCLE_TIME_DECREASE)
        {
          if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) == NODE_BEHAVIOUR_TYPE_UNICAST){

            set_cycletime(MIN(ENERGY_EFFICIENCY_GET(lad_rdc_cycle_time),ENERGY_EFFICIENCY_GET(red_rdc_cycle_time)));
            // printf("NODE_BEHAVIOUR_TYPE_UNICAST\n" );
          }

        }

        if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) == NODE_BEHAVIOUR_TYPE_NOTHING)
        {

            set_cycletime(ENERGY_EFFICIENCY_GET(rdc_cycle_time));

        }
      }

      if(plan == PLAN_B){

        if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) != NODE_BEHAVIOUR_TYPE_UNICAST){

          set_cycletime(ENERGY_EFFICIENCY_GET(con_rdc_cycle_time));

        }

        if(ENERGY_EFFICIENCY_GET(cycle_time_direction) == CYCLE_TIME_INCREASE)
        {
          if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) == NODE_BEHAVIOUR_TYPE_NOTHING)
          {

            set_cycletime(ENERGY_EFFICIENCY_GET(rdc_cycle_time));

          }

        }

        if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) == NODE_BEHAVIOUR_TYPE_UNICAST){

          set_cycletime(MIN(ENERGY_EFFICIENCY_GET(lad_rdc_cycle_time),ENERGY_EFFICIENCY_GET(red_rdc_cycle_time)));
          // printf("NODE_BEHAVIOUR_TYPE_UNICAST\n" );
        }

      }


      if(plan == PLAN_C){

        if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) != NODE_BEHAVIOUR_TYPE_UNICAST){

          set_cycletime(ENERGY_EFFICIENCY_GET(con_rdc_cycle_time));

        }

        if(ENERGY_EFFICIENCY_GET(cycle_time_direction) == CYCLE_TIME_INCREASE)
        {
          if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) == NODE_BEHAVIOUR_TYPE_NOTHING)
          {

            set_cycletime(ENERGY_EFFICIENCY_GET(rdc_cycle_time));

          }

        }

        if(energy_efficiency_get_current_node_behauiour(real_time_clock_second()) == NODE_BEHAVIOUR_TYPE_UNICAST)
        {

            set_cycletime(ENERGY_EFFICIENCY_GET(rdc_cycle_time));
        
        }

      }
    }else{
      reset_behaviour_tb_index();
      set_cycletime(ENERGY_EFFICIENCY_GET(con_rdc_cycle_time));
      ENERGY_EFFICIENCY_SET_X_EQ_Y(energy_efficiency_effect,ENERGY_EFFICIENCY_INVALID);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

void 
rdc_efficiency_request_poll(void)
{
  process_poll(&rdc_efficiency_process);
}
/*---------------------------------------------------------------------------*/
void
rdc_efficiency_init(void)
{
  ENERGY_EFFICIENCY_INIT(280818,RTIMER_ARCH_SECOND/get_rdc_active_channel_check_rate(),10000);
  process_start(&rdc_efficiency_process, NULL); 

}
/*---------------------------------------------------------------------------*/
