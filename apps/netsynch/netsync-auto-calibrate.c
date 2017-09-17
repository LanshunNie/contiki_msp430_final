/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: netsync-auto-calibrate.c ,v 1.21 2016/12/05 13:46:15 
*/
/**
 * \file
 *        net time sync auto calibrate mechanism.
 * \author
 *        @yang
 */

#include "netsync-auto-calibrate.h"
#include "contiki.h"
#include "sys/clock.h"

#define DEBUG  0  //DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define MIN_CAL_OFFSET  12
#define MAX_CAL_OFFSET  480
#define CAL_ERROR_DIFF  2

static struct netsync_cal_s netsync_cal_info;
/*---------------------------------------------------------------------------*/
static uint32_t 
get_time_total_second(soft_time s_time){
  return (s_time.hour * 3600 + s_time.minute * 60 +s_time.sec);
}
/*---------------------------------------------------------------------------*/
static uint8_t
abs_greater_u16(int v,int limit_v){
  return (v > limit_v || v < -limit_v);
}
/*---------------------------------------------------------------------------*/
static uint8_t
abs_lesser_u16(int v,int limit_v){
  return (v < limit_v && v > -limit_v);
}
/*---------------------------------------------------------------------------*/
struct netsync_cal_s *
get_autocal_info(){
  return (&netsync_cal_info);
}
/*---------------------------------------------------------------------------*/
static uint8_t 
netsyc_offset_correct(int netsyc_offset){
  return (abs_greater_u16(netsyc_offset,MIN_CAL_OFFSET)&&abs_lesser_u16(netsyc_offset,MAX_CAL_OFFSET));
}
/*---------------------------------------------------------------------------*/
/*compute neighbor two cal time interval. */

static uint32_t
netsync_autocal_neighbor_interval(){
  soft_time times;
  get_timenow(&times);
  uint32_t now_time_total_second = get_time_total_second(times);
  uint32_t last_caltime_total_second = get_time_total_second(netsync_cal_info.last_caltime);

  uint32_t time_interval = (get_nowdays() - netsync_cal_info.days) * 86400 + now_time_total_second - last_caltime_total_second;
  return time_interval;
}
/*---------------------------------------------------------------------------*/
static void
netsync_compute_autocal_interval(int netsyc_offset,uint32_t cal_interval){
  if(netsyc_offset > 0 && netsyc_offset > CAL_ERROR_DIFF){
    netsync_cal_info.cal_offest = 1;
    netsync_cal_info.autocal_interval =  cal_interval / (netsyc_offset - CAL_ERROR_DIFF) + 1;   //enlarge interval.
  }else if(netsyc_offset < 0 && (-netsyc_offset) > CAL_ERROR_DIFF){
    netsync_cal_info.cal_offest = -1;
    netsync_cal_info.autocal_interval =  cal_interval / ((-netsyc_offset) + CAL_ERROR_DIFF) + 1;
  }else{
    netsync_cal_info.cal_offest = 0;
    netsync_cal_info.autocal_interval = 0;
  }
}
/*---------------------------------------------------------------------------*/
#if 0
static void
netsync_autocal_update_interval(int new_netsyc_offset,uint32_t cal_interval){
  uint16_t multi_times = cal_interval / netsync_cal_info.autocal_interval;
  int netsyc_offset =multi_times * (netsync_cal_info.cal_offest) + new_netsyc_offset;
 
  netsync_compute_autocal_interval(netsyc_offset,cal_interval);  
}
#endif
/*---------------------------------------------------------------------------*/
void 
netsync_autocal_update(int netsyc_offset){
  static uint32_t new_cal_interval;
  static int new_netsyc_offset;

  if(netsyc_offset_correct(netsyc_offset)){
    new_cal_interval = netsync_autocal_neighbor_interval();
    netsync_cal_time_record();
    if(netsync_cal_info.init_flag){
      new_netsyc_offset =(new_cal_interval / netsync_cal_info.autocal_interval) * (netsync_cal_info.cal_offest) + netsyc_offset;
      netsync_compute_autocal_interval(new_netsyc_offset,new_cal_interval);  
    }else{
      netsync_cal_info.init_flag = 1;
      netsync_compute_autocal_interval(netsyc_offset,new_cal_interval);
    }
    set_autocal_info(netsync_cal_info.cal_offest,netsync_cal_info.autocal_interval);
  }
}
/*---------------------------------------------------------------------------*/
void 
netsync_cal_time_record(void){
  soft_time times;
  get_timenow(&times);
  netsync_cal_info.last_caltime = times;
  netsync_cal_info.days = get_nowdays();
}
/*---------------------------------------------------------------------------*/
void
netsync_autocal_init(void){
  memset(&netsync_cal_info, 0, sizeof(netsync_cal_info));
}
/*---------------------------------------------------------------------------*/

