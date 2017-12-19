/**
 * \addtogroup netsynch
 * @{
 */


/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: netsynch.c,v 1.21 2016/1/26 22:47:38 
 */

/**
 * \file
 *         A simple time synchronization mechanism
 * \author
 *         zhangwei
 */
#if 1
#include "netsynch.h"
#include "contiki.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/rpl/rpl.h"
#include "dev/serial-line.h"
#include "net/linkaddr.h"

#include "net/netstack.h"
#include "node-id.h"

#include <stdio.h>
#include <string.h>

#include "contiki-lib.h"
#include "contiki-net.h"
#include "simple-udp.h"

#include "netsync-auto-calibrate.h"
 
#include "lib/random.h"
#include "task-schedule.h"

#if UIP_CONF_IPV6_RNFD
#include "net/rnfd/rnfd.h"
#endif

#if UIP_CONF_MULTI_SUBNET
#include "multi-subnet.h"
#endif

#define DEBUG  0//DEBUG_PRINT
#include "net/ip/uip-debug.h"
#define MAX_PAYLOAD_LEN 120

#define MCAST_NETSYNCH_UDP_PORT 6103 /* Host byte order */
static struct uip_udp_conn * mcast_conn;

#define NUM_LT(a,b) ((int8_t)(((uint8_t)(a)) - ((uint8_t)(b)))) < 0

#if ROOTNODE
    #define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
    #define UDP_PAN_PORT 6102
    #define UDP_ROOT_PORT 6104
    static struct uip_udp_conn * pan_connection;
#endif

#define AUTHORITY_LEVEL_UNSYNCHED 255

static int authority_level = AUTHORITY_LEVEL_UNSYNCHED;

static uint8_t schedule_bitmap[18];
static uint8_t schedule_bitmap_new[18];

static void netsynch_send(void * p);
static void prepare_mcast(void);
static void netsynch_receiver(struct netsynch_msg msg);
//static void init_schedule(void);

#if ROOTNODE
    static void sent_to_pan(void * p);
    static void request_to_pan(void);
    static struct task_schedule to_pan_ts;
    static int netsynch_period =1;

    static uint8_t netsync_init_flag = 0;
#else
    static struct ctimer common_request_ct; 
    static void request_send(void *p);
#endif
    
static struct etimer periodic;

static uint8_t last_seqnum = 255;
#define PERIOD (CLOCK_SECOND *60*5)

#if ROOTNODE
#define REQUEST_PERIOD (60* CLOCK_SECOND) //15*CLOCK_SECOND
#else
#if UIP_CONF_AUTO_SLEEP
#define REQUEST_PERIOD (120* CLOCK_SECOND) //15*CLOCK_SECOND  //110
#else
#define REQUEST_PERIOD (180* CLOCK_SECOND) //15*CLOCK_SECOND  //110
#endif
#endif

#define NETSYNC_SEQ_DIFF_MAX   4   //4
#define HEART_ALARM 10      //15
#define FWD_DELAY()  (NETSTACK_RDC.channel_check_interval())

//clock_time_t  period =   ( CLOCK_SECOND *60*5);
//clock_time_t request_period = (3*60*  CLOCK_SECOND);

// static struct task_schedule root_task_ts;
static int netsyn_offset = 0;

//static int others_synch = 0;
static int  response=0;
static uip_ipaddr_t concentrator_ipv6addr;
static struct netsynch_msg rev_msg;
static struct  ctimer send_netsync_ct;

/*---------------------------------------------------------------------------*/
PROCESS(netsynch_process, "Netsynch process");

/*---------------------------------------------------------------------------*/
#if !ROOTNODE
static clock_time_t
random_delay(){
  return  (CLOCK_SECOND/2 + random_rand()%((FWD_DELAY()*uip_ds6_nbr_num()+12*CLOCK_SECOND)));
}
#endif
/*---------------------------------------------------------------------------*/
int
netsynch_is_synched(void)
{
  return authority_level < AUTHORITY_LEVEL_UNSYNCHED;
}
/*---------------------------------------------------------------------------*/
#if !ROOTNODE
static uint8_t
abs_diff_value_greater_u8(uint8_t v1,uint8_t v2,uint8_t limit_v){
 if(v1 > v2 && v1 - v2 > limit_v)
  return 1;
 if(v2 > v1 && v2 - v1 > limit_v)
  return 1;
 return 0;
}
#endif
/*---------------------------------------------------------------------------*/
int
netsynch_authority_level(void)
{
  return authority_level;
}
/*---------------------------------------------------------------------------*/
void
netsynch_set_authority_level(int level)
{
  //int old_level = authority_level;
 
 #if 0 
  #if ROOTNODE & 0
      if( level != 1 ){

        level=AUTHORITY_LEVEL_UNSYNCHED;
        init_schedule();
       }

 #endif
#endif

  authority_level = level;

//  if(old_level != authority_level) {

//    process_exit(&netsynch_process);

//    process_start(&netsynch_process,NULL);
 // }
}
/*---------------------------------------------------------------------------*/
void netsynch_time(void)
{
  soft_time  caltime;
  get_timenow(&caltime);
}
/*---------------------------------------------------------------------------*/
static void
adjust_caltime(soft_time cal_time)
{
  syn_update_timenow(cal_time);
}
/*---------------------------------------------------------------------------*/
void schedule_bitmap_set_new(uint8_t bitmap[]){

  memcpy(schedule_bitmap_new,bitmap, sizeof(schedule_bitmap) );

}
/*---------------------------------------------------------------------------*/
void update_new_schedule(void*p){

  memcpy(schedule_bitmap,schedule_bitmap_new, sizeof(schedule_bitmap) );

}

// void update_schedule(uint8_t bitmap[]){
void update_schedule(void *p){

  uint8_t *bitmap = (uint8_t *)(p);
  // int i=0;
  memcpy(schedule_bitmap,bitmap, sizeof(schedule_bitmap) );

#if ROOTNODE | DEBUG
    printf(" sche_map :\n"); 
#endif
    
}
/*---------------------------------------------------------------------------*/

void schedule_bitmap_get(uint8_t *bitmap){
  memcpy(bitmap,schedule_bitmap, sizeof(schedule_bitmap));
}
/*---------------------------------------------------------------------------*/

void netsynch_cal_offset(soft_time cal_time){
  soft_time times;
  get_timenow(&times);
  netsyn_offset = (times.hour-cal_time.hour) * 3600 + (times.minute - cal_time.minute)*60 + (times.sec-cal_time.sec);
}

void set_recvmsg(struct netsynch_msg msg){
  rev_msg.seqnum=msg.seqnum;
  rev_msg.authority_level=msg.authority_level;
  rev_msg.caltime = msg.caltime;
  rev_msg.nodeID=msg.nodeID;
}

void get_recvmsg(struct netsynch_msg *msg){
  msg->seqnum=rev_msg.seqnum;
  msg->authority_level=rev_msg.authority_level;
  msg->caltime=rev_msg. caltime;
  msg->nodeID=rev_msg.nodeID;
}
/*---------------------------------------------------------------------------*/
int netsynch_get_offset(void){
  return netsyn_offset;
}
/*---------------------------------------------------------------------------*/
uint8_t get_lastseqnum(void){
  return last_seqnum;
}
/*---------------------------------------------------------------------------*/

static 
void
netsynch_receiver(struct netsynch_msg msg)
{
//  static struct task_schedule ts;

#if ROOTNODE

  if(uip_ipaddr_cmp(&UIP_IP_BUF->srcipaddr, get_concentrator_ipv6addr())){
     if((msg.authority_level) <= authority_level){
        netsynch_cal_offset(msg.caltime);

        if(netsyn_offset >= 5 ||netsyn_offset <= -5){
          ctimer_set(&send_netsync_ct,(3*CLOCK_SECOND),sent_to_pan,NULL);
          return;
        }

          adjust_caltime(msg.caltime);

          schedule_bitmap_set_new(msg.sche_bitmap);

          //netsynch_set_authority_level((msg.authority_level) + 1);
          netsynch_set_authority_level(1);
//          printf("levelllllll:%d\n", authority_level);
          set_recvmsg(msg);

          ++last_seqnum;
        
        // if(others_synch==0){
        //    etimer_set(&periodic,PERIOD);
        //  }
          if(netsync_init_flag){
            if(get_idle_time() >60){
          
             ctimer_set(&send_netsync_ct,15*CLOCK_SECOND,netsynch_send,NULL);
          //  netsynch_send(NULL);
         //   task_schedule_set(&ts,TEMP_TASK,TASK_READY,TASK_PERIOD_DEFAULT,netsynch_send,NULL);
            }
          }else{
            netsync_init_flag =1;
            task_schedule_set(&to_pan_ts,MUST_TASK,TASK_READY,netsynch_period,sent_to_pan,NULL); 
          }
      }
  }else{
    if((msg.authority_level) == AUTHORITY_LEVEL_UNSYNCHED && netsynch_is_synched() && response==0){
        
      response=1;

#if SUBNET_PANID_CONF_LIMIT & 0
    /* drop not our panid netsync message */
      if(get_link_receive_subnet_panid() != get_own_subnet_panid() && get_link_receive_subnet_panid() != get_default_subnet_panid()) {
        return;
      }
#endif    

      ctimer_set(&send_netsync_ct,CLOCK_SECOND,netsynch_send,NULL);

      }
    // if(authority_level == 1 && others_synch ==0 && (msg.authority_level) < AUTHORITY_LEVEL_UNSYNCHED){
    //     others_synch =1;
    //     printf("saa\n");
    //     etimer_stop(&periodic);
    //     task_schedule_set(&to_pan_ts,MUST_TASK,TASK_READY,netsynch_period,sent_to_pan,NULL);   
    //   }
  }
  
#else
    uint8_t netsync_flag =0;
    int old_netsync_offest =0;
    uint8_t rnfd_offline_flag = 0;

#if SUBNET_PANID_CONF_LIMIT    
    //printf("receive panid:%04x\n",get_link_receive_subnet_panid());
#endif

   if(netsynch_is_synched()){
    if(msg.authority_level< AUTHORITY_LEVEL_UNSYNCHED){
      
#if SUBNET_PANID_CONF_LIMIT 
    /* drop not our panid netsync message */
      if(get_own_subnet_panid() != get_link_receive_subnet_panid()) {
        return;
      }
#endif

      if(last_seqnum == msg.seqnum)
        return;

#if UIP_CONF_IPV6_RNFD
    rnfd_offline_flag = get_rnfd_offline_nodag();
#endif
      old_netsync_offest = netsyn_offset;
     
     
      if(msg.authority_level <= netsynch_authority_level() && rnfd_offline_flag == 0 &&
        netsyn_offset <= 480 && netsyn_offset >= -480){
        netsync_flag = 1;
      }

        netsyn_offset = old_netsync_offest;
      
      if(netsync_flag ||(msg.authority_level <= netsynch_authority_level()&&
        (NUM_LT(last_seqnum,msg.seqnum)||
          abs_diff_value_greater_u8(last_seqnum,msg.seqnum,NETSYNC_SEQ_DIFF_MAX)))){
          
          netsynch_cal_offset(msg.caltime);

          adjust_caltime(msg.caltime);

          schedule_bitmap_set_new(msg.sche_bitmap);

          if(rnfd_offline_flag == 0 || msg.authority_level < netsynch_authority_level()){
            netsynch_set_authority_level((msg.authority_level) + 1);
          }

         // printf("receive netsync level:%d\n", msg.authority_level);

          set_recvmsg(msg);
          last_seqnum = (msg.seqnum);
      
          ctimer_stop(&send_netsync_ct);
          ctimer_set(&send_netsync_ct,random_delay(),netsynch_send,NULL);

          netsync_autocal_update(netsyn_offset);
      //    task_schedule_set(&ts,TEMP_TASK,TASK_READY,TASK_PERIOD_DEFAULT,netsynch_send,NULL);
      }    
    }else if((msg.authority_level) == AUTHORITY_LEVEL_UNSYNCHED && response==0){
 
#if SUBNET_PANID_CONF_LIMIT & 0
    /* drop not our panid netsync message */
      if(get_link_receive_subnet_panid()!= get_own_subnet_panid() && get_link_receive_subnet_panid() != get_default_subnet_panid()) {
        return;
      }
#endif   

        response=1;
        ctimer_set(&send_netsync_ct,random_delay(),netsynch_send,NULL);
       // task_schedule_set(&ts,TEMP_TASK,TASK_READY,TASK_PERIOD_DEFAULT,netsynch_send,NULL);
    }
  }else{
    if((msg.authority_level) < AUTHORITY_LEVEL_UNSYNCHED){
      // netsynch_cal_offset(msg.caltime);

#if SUBNET_PANID_CONF_LIMIT & 0
      /* not our panid netsync message */
      if(get_link_receive_subnet_panid() != get_own_subnet_panid() && get_own_subnet_panid() != get_default_subnet_panid()) {
        return;
      }
      //set_own_subnet_panid(get_link_receive_subnet_panid());
#endif
          adjust_caltime(msg.caltime);

          schedule_bitmap_set_new(msg.sche_bitmap);

          netsynch_set_authority_level((msg.authority_level) + 1);
//          printf("levelllllll:%d\n", authority_level);
          set_recvmsg(msg);
          last_seqnum = (msg.seqnum);
 
          ctimer_set(&send_netsync_ct,random_delay(),netsynch_send,NULL);
         // task_schedule_set(&ts,TEMP_TASK,TASK_READY,TASK_PERIOD_DEFAULT,netsynch_send,NULL);
          netsync_cal_time_record();
    }
  }
#endif
}

/*---------------------------------------------------------------------------*/
static void
netsynch_send(void * p)
{
  if(netsynch_is_synched()){
  uip_ipaddr_t ipaddr;
  static struct ctimer back_ct;

  response=0;
  struct netsynch_msg msg;
  ctimer_set(&back_ct,30*CLOCK_SECOND,update_new_schedule,NULL);
   
  if(get_idle_time() > 4){

    msg.seqnum = (last_seqnum);
    msg.authority_level = (authority_level);
    get_timenow(&(msg.caltime));
    memcpy(msg.sche_bitmap, schedule_bitmap_new, sizeof(schedule_bitmap_new) );

    msg.nodeID=node_id;
    uip_create_linklocal_allnodes_mcast(&ipaddr);
  
#if ROOTNODE | DEBUG
    printf("send time :%d,%d,%d\n",msg.caltime.hour,msg.caltime.minute,msg.caltime.sec);  
    printf("send seq:%d,level:%d\n",msg.seqnum,authority_level);

#endif
    uip_udp_packet_sendto(mcast_conn, &msg, sizeof(msg),
                       &ipaddr, UIP_HTONS(MCAST_NETSYNCH_UDP_PORT));
   }
  } 
}
/*---------------------------------------------------------------------------*/
#if !ROOTNODE
static void request_send(void *p){

  uip_ipaddr_t ipaddr;
  struct netsynch_msg msg;
   
  if(!netsynch_is_synched()){
    msg.seqnum = (last_seqnum);
    msg.authority_level =( AUTHORITY_LEVEL_UNSYNCHED);
    get_timenow(&(msg.caltime));
    memcpy(msg.sche_bitmap, schedule_bitmap, sizeof(schedule_bitmap));
    
    msg.nodeID=node_id;
    uip_create_linklocal_allnodes_mcast(&ipaddr);

  // printf("request time :%d,%d,%d\n",msg.caltime.hour,msg.caltime.minute,msg.caltime.sec);   
    ctimer_set(&common_request_ct,REQUEST_PERIOD,request_send, NULL);

    uip_udp_packet_sendto(mcast_conn, &msg, sizeof(msg),
                       &ipaddr, UIP_HTONS(MCAST_NETSYNCH_UDP_PORT));
  }
}
#else 
/*---------------------------------------------------------------------------*/

static void sent_to_pan(void * p){

  //uip_ipaddr_t ipaddr;

   struct netsynch_msg msg;
    msg.seqnum = (last_seqnum);
    msg.authority_level = (authority_level);
    get_timenow(&(msg.caltime));
    memcpy(msg.sche_bitmap, schedule_bitmap_new, sizeof(schedule_bitmap_new) );
    msg.nodeID=node_id;
 // uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0x0002);

  // printf("sent_to_pan.\n");
  // printf("sent_to_pan time :%d,%d,%d\n",msg.caltime.hour,msg.caltime.minute,msg.caltime.sec);   

  uip_udp_packet_sendto(pan_connection, &msg, sizeof(msg),
                       get_concentrator_ipv6addr(), UIP_HTONS(UDP_PAN_PORT));

}
/*---------------------------------------------------------------------------*/
static void request_to_pan(void){
 // uip_ipaddr_t ipaddr;

    struct netsynch_msg msg;
    msg.seqnum = (last_seqnum);
    msg.authority_level =( AUTHORITY_LEVEL_UNSYNCHED);
    get_timenow(&(msg.caltime));

    memcpy(msg.sche_bitmap, schedule_bitmap, sizeof(schedule_bitmap) );
    msg.nodeID=node_id;
 // uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0x0002);
  //printf("request_to_pan.\n");
  // printf("request_to_pan time :%d,%d,%d\n",msg.caltime.hour,msg.caltime.minute,msg.caltime.sec);   

  uip_udp_packet_sendto(pan_connection, &msg, sizeof(msg),
                       get_concentrator_ipv6addr(), UIP_HTONS(UDP_PAN_PORT));
}
#endif
/*---------------------------------------------------------------------------*/
static void
prepare_mcast(void){
  mcast_conn = udp_new( NULL, UIP_HTONS(MCAST_NETSYNCH_UDP_PORT), NULL);
  udp_bind(mcast_conn, UIP_HTONS(MCAST_NETSYNCH_UDP_PORT));

#if ROOTNODE
  pan_connection = udp_new( NULL, UIP_HTONS(UDP_PAN_PORT), NULL);
  udp_bind(pan_connection , UIP_HTONS(UDP_ROOT_PORT));
#endif
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {
    struct netsynch_msg msg;
    uint8_t  non_frag_flag =0;
  #if ROOTNODE

    int i=0;
    if(uip_ipaddr_cmp(&UIP_IP_BUF->srcipaddr, get_concentrator_ipv6addr())){
	
    // if((UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2])==0 && 
    //   (UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1])==2){

      msg.seqnum = ( ((uint8_t * )uip_appdata)[0]);

      msg.authority_level =( ((uint8_t * )uip_appdata)[1]);

      msg.caltime.hour = (((uint8_t *)uip_appdata)[2]);

      msg.caltime.minute = (((uint8_t *)uip_appdata)[3]);

      msg.caltime.sec = (((uint8_t *)uip_appdata)[4]);
  
      netsynch_period = (((uint8_t *)uip_appdata)[5]);

      task_schedule_set_period(&to_pan_ts,netsynch_period);
      // printf("netsynch_period = %d\n",netsynch_period );

 // memcpy(msg.sche_bitmap, uip_appdata+6, sizeof(msg.sche_bitmap));
      for(i=6;i<SCHEDULE_SIZE+6;i++){

        msg.sche_bitmap[i-6] = (((uint8_t *)uip_appdata)[i]);
      }

    }else{
     non_frag_flag = uip_udp_received_data_preprocess();
     if(non_frag_flag){
        memcpy(&msg, uip_appdata, sizeof(msg));
     }else{
      return;
     }

    }
    // printf("netsynch_period = %d\n",netsynch_period );

#else
     non_frag_flag = uip_udp_received_data_preprocess();
     if(non_frag_flag){
      memcpy(&msg, uip_appdata, sizeof(msg));
       // printf("level:%u\n",msg.authority_level);

     }else{
      return;
    }
#endif 

  #if ROOTNODE | 1 //DEBUG 
    printf("\n");
    printf("receive time :%d,%d,%d,%d\n",msg.caltime.hour,msg.caltime.minute,msg.caltime.sec,msg.authority_level);   
  #endif

    netsynch_receiver(msg);
  }
}

/*------------------------------------------------------------------------*/
#if UIP_CONF_IPV6_RNFD  
static uint16_t
abs_value_u16(int value){
  if(value < 0){
     return (-value);
  }

  return value;
}
/*------------------------------------------------------------------------*/
static void 
netsynch_common_send(void *p){
  ++last_seqnum;
  printf("netsynch common send\n");
  
  netsynch_send(NULL);
}
/*------------------------------------------------------------------------*/
void
netsync_common_timesource(void){
  static uint8_t count = 0;
  uint16_t abs_netsyn_offset;

  abs_netsyn_offset = abs_value_u16(netsyn_offset);

  if(abs_netsyn_offset > 5)
    return;

  ++count;
  clock_time_t netsyn_send_delay;

  if(count % 2 == 0 && netsynch_authority_level() == 2){
    netsyn_send_delay = (8 + abs_netsyn_offset*12)* CLOCK_SECOND + random_rand()%(6* CLOCK_SECOND); 
    ctimer_set(&send_netsync_ct,netsyn_send_delay,netsynch_common_send,NULL);
  }
}
#endif
/*------------------------------------------------------------------------*/
#if 1
void 
init_concentrator_ipv6addr(void){
   uip_ip6addr(&concentrator_ipv6addr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0x0002);
}

uip_ipaddr_t* 
get_concentrator_ipv6addr(void){
  return &concentrator_ipv6addr;
}

void 
set_concentrator_ipv6addr(const uip_ipaddr_t *ip6addr){
   uip_ip6addr_copy(&concentrator_ipv6addr,ip6addr); 
}
#endif

void 
init_schedule(void){
    int i=0;
    for(i=0;i< sizeof(schedule_bitmap);i++){
        schedule_bitmap[i]=0xFF;
    }

#if 0
    PRINTF(" sche_map :\n");     /*   */
    for (i = 0; i < 18; ++i){     
      PRINTF("i:%d, %x ",i,schedule_bitmap[i]);
    }
#endif

}

uint8_t 
netsync_schedule_all_on(void){
#if 0
    printf("sche_map2 :\n");     /*   */
    int j=0;
    for (j = 0; j < 18; ++j){     
      printf("j:%d, %x ",j,schedule_bitmap[j]);
    }
#endif

  int i=0;
  for(i=0;i< sizeof(schedule_bitmap);i++){
    if(schedule_bitmap[i] != 0xFF){
      //printf("on:00000\n");
      return 0;
    }  
  }
  //printf("on:111111\n");

  return 1;
}

/*---------------------------------------------------------------------------*/
#if !ROOTNODE
void
netsync_start_common_request(void) {
  authority_level = AUTHORITY_LEVEL_UNSYNCHED;
  last_seqnum = 0xff;
  
  ctimer_set(&common_request_ct,CLOCK_SECOND*5 + random_rand()%(10*CLOCK_SECOND),request_send, NULL);
}
#endif
/*---------------------------------------------------------------------------*/
void
netsynch_init(void){

  init_schedule();

#if ROOTNODE 
  last_seqnum = random_rand();

#else
  last_seqnum = 0xff;

#endif

  init_concentrator_ipv6addr();

  process_start(&netsynch_process,NULL);  

  netsync_autocal_init();

#if (!ROOTNODE) & (!SUBNET_PANID_CONF_TIMESLOT_DIFF)
  netsync_start_common_request();
#endif
// #if ROOTNODE & DEBUG
//   PRINTF("ROOTNODE-------------------------------------\n");
// #endif
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(netsynch_process, ev, data)
{

  PROCESS_BEGIN();
  PROCESS_PAUSE();
  prepare_mcast();

  if(!netsynch_is_synched()){
    etimer_set(&periodic,REQUEST_PERIOD);
  }

  while(1) {

    PROCESS_YIELD();
  
   if(etimer_expired(&periodic)) {
      
      // if(authority_level==0 && others_synch==0){

      // if(authority_level==1 && others_synch==0){

      //   etimer_set(&periodic,PERIOD);
      //   netsynch_send(NULL); 
      // }

     if(!netsynch_is_synched()){
        #if ROOTNODE
          etimer_set(&periodic,REQUEST_PERIOD);
          request_to_pan();
        #endif
      }
    }

   if(ev == tcpip_event) {
       tcpip_handler();
     }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#endif
