/*
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
 */
#include "contiki.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "contiki-net.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "net/rpl/rpl.h"
#include "dev/serial-line.h"
#include "simple-udp.h"
#include <stdlib.h>
//#include "dev/uart1.h"
//#include "dev/uart0.h"
#if HW_CONF_WITH_UART1
#include "dev/uart1.h"
#else
#include "dev/uart0.h"
#endif

#include "dev/leds.h"
 
#include "net/ip/uip-debug.h"
#include <stdio.h>
#include <string.h>
#include "lib/ringbuf.h"
#include "simple-udp.h"
#include "util.h"
#include "node-id.h"
#include "sys/ctimer.h"

#include "task-schedule.h"
 
#include "netsynch.h"

#include "node_function.h"

//#define UIP_MCAST6_CONF_ENGINE UIP_MCAST6_ENGINE_SCF
#define MAX_BUFFER_LENGTH 127

//port area
#define UDP_CLIENT_PORT 8775//
#define UDP_SERVER_PORT 5688//
#define UDP_SERVER_DATA_PORT 8765
#define MCAST_SINK_UDP_PORT 3001 
#define UDP_SERVER_UNICAST_PORT 5656

#define MCAST_TO_WHOLE_NETWORK       1 //
#define MCAST_TO_SOME_NODE   2 //

#define state_ready 0

#define DEBUG 0//DEBUG_PRINT
//#define SEND_BASE 

#include "net/ip/uip-debug.h"
#define MAX_PAYLOAD_LEN   255
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *client_conn_data;

static uip_ipaddr_t server_ipaddr;
static struct uip_udp_conn *sink_conn;

static char global_type;
static char ack_buffer[MAX_PAYLOAD_LEN];

static unsigned char appdata_length;

static int n_id;

static struct task_schedule read_data_ts;

static struct task_schedule heart_ts;

// static struct task_schedule temp_task_ts;

// static void node_send(void * p);
 
static struct ctimer ct;
static struct ctimer send_ct;
static void  msg_handler(char *appdata,int appdata_length);


static int ctl_content_length;

bool uploadFlag;

static int data_length;
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
#define FWD_DELAY()  (NETSTACK_RDC.channel_check_interval())
static clock_time_t
random_delay(){
  return  (CLOCK_SECOND*30 + random_rand()%((FWD_DELAY()*uip_ds6_nbr_num()*3/2)+8*CLOCK_SECOND));
}

 static  char msg_da[]= {0x01,0x80,0x68,0x36,0x36,0x68,0x08,0x02,0x72,0x00,0x00,0x00,0x00,0x8F,0x41,0x03,0x22,0x02,0x00,0x00,0x00,0x0F,0x07,0x00,0xF1,0x08,0x04,0x23,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x35,0x23,0x10,0x19,0x08,0x23,0x0A,0x15,0x20,0x05,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,
    0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0x16,0x00,0x02,0x00,0x00,0x00,0x00,0xFF,0xFF}; 

static void sendCmd(void *p)
{
     
    int i;
    
    uint8_t* cmd = (uint8_t *)(p);
    
   // leds_toggle(LEDS_GREEN);   

    for(i=0;i< cmd[1];i++)
    {
        putchar((unsigned char )cmd[i+2]);
    }
 
    msg_da[2]=netsynch_authority_level();

    uip_udp_packet_sendto(client_conn_data, msg_da, sizeof(msg_da),
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
 
}
/*---------------------------------------------------------------------------*/
static void send_read_meter_cmd(void *p)
{
      int i;
   //  // printf("meter cmd is:");
    // print_cmd_array();
   //   printf("cmd[0] is: %x",cmd_read_meter[0]);

   // leds_toggle(LEDS_GREEN);   

   if(cmd_read_meter[0] != 0xFF)
   {
        // printf("cmd is right burned \n");
            for(i=0;i< cmd_read_meter[1];i++)
            {
                putchar((unsigned char )cmd_read_meter[i+2]);
            }
            
 
        msg_da[2]=netsynch_authority_level();

          uip_udp_packet_sendto(client_conn_data, msg_da, sizeof(msg_da),
                             &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
    }

}

/*---------------------------------------------------------------------------*/
//shen me dou bu hui "h"
// static void node_send(void * p){

//       // soft_time times;

//       // printf("node_send*****\n");

//       // get_timenow(&times);

//       //  ack_buffer[2] = 'Y';
//        uip_udp_packet_sendto(client_conn_data, "h", 1,
//                       &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
       
//        // uip_udp_packet_sendto(client_conn_data, ack_buffer, 3,//appdata_length
//        //                &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
      
      
// }
 //task_schedule_set(&temp_send,TEMP_TASK,TASK_READY,node_send,NULL);//data
       // ctimer_set(&ct,60 * CLOCK_SECOND * 3+random_rand() % (5 * CLOCK_SECOND),node_send,NULL);

/*---------------------------------------------------------------------------*/
static uint8_t
contain_node(uint8_t data_len,char *data){

  uint8_t contain_flag = 0;
#if 0
  int i=0;
  for(i=0;i<data_len;i++){
    printf("%02x,",data[i]);
  }
  printf("\n");
#endif
  int i = 0;
  linkaddr_t *addr = NULL;
  if(data_len % 8 == 0){
  	 for(i= 0;i< data_len/8;i++){
  		memcpy(addr,data+i*8,8);
  		if(linkaddr_cmp(addr,&linkaddr_node_addr)){
     		contain_flag =1;
     		break;
    	}
  	}
  }
    
  return contain_flag;
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) 
  {
   // printf("1010101010\n");
  // leds_toggle(LEDS_GREEN); 
    uip_udp_received_data_preprocess();
    appdata_length = (unsigned char)((char *)uip_appdata)[0];
    char *appdata = (char *)malloc((appdata_length +1) * sizeof(char));
    memcpy(appdata,(char *)uip_appdata + 1,appdata_length);
    appdata[appdata_length] = '\0';

//    unsigned char ctl_content_length = 0;
    memset(ack_buffer,0,MAX_PAYLOAD_LEN);

    global_type = appdata[0];
    ack_buffer[0] = global_type;
    ack_buffer[1] = appdata[1];

     switch(global_type)
     {
      //printf("mcast type :%x\n",appdata[1] );

      case MCAST_TO_WHOLE_NETWORK://duobo chun zhiling
          msg_handler(appdata+1,appdata_length-1);
      break;

      case MCAST_TO_SOME_NODE: 
        ctl_content_length = (unsigned char) appdata[1];
        if( contain_node(appdata_length - 2 - ctl_content_length,appdata + 2))
        {
       //  leds_toggle(LEDS_ALL);
      //    printf("receive upload\n");   
          msg_handler(appdata+34,ctl_content_length);

        }
      break;
      }

    free(appdata);
  }
}

/*---------------------------------------------------------------------------*/
void
system_monitor_msg_send(void  * p)
{      
    uint8_t msg[SYSTEM_MONITOR_MSG_LENGTH];
 
    get_system_monitor_msg(msg,SYSTEM_MONITOR_MSG_LENGTH);

    uip_udp_packet_sendto(client_conn, &msg, sizeof(msg),
                    &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

}

// void
// system_monitor_msg1_send(void  * p)
// {
//        uint8_t msg1[SYSTEM_MONITOR_MSG_LENGTH1];
 
//        get_system_monitor_msg1(msg1,SYSTEM_MONITOR_MSG_LENGTH1);

//        memcpy(ack_buffer+2,msg1,SYSTEM_MONITOR_MSG_LENGTH1);
       
//        uip_udp_packet_sendto(client_conn_data, &ack_buffer,SYSTEM_MONITOR_MSG_LENGTH1+2,
//                         &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
// }
/*---------------------------------------------------------------------------*/

static void
set_global_address(void)
{
  // uip_ipaddr_t ipaddr;

  // uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  // uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  // uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  /* set server address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0x0002);

}
static uip_ds6_maddr_t *
join_mcast_group(void)
{
  uip_ipaddr_t addr;
  uip_ds6_maddr_t *rv;

  /* First, set our v6 global */
  uip_ip6addr(&addr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&addr, &uip_lladdr);
  uip_ds6_addr_add(&addr, 0, ADDR_AUTOCONF);
  uint8_t state;
  int i;
   for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
    }
  }

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
  uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);
  rv = uip_ds6_maddr_add(&addr);

  if(rv) {
    PRINT6ADDR(&uip_ds6_maddr_lookup(&addr)->ipaddr);
  }
  return rv;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  PROCESS_BEGIN();


  // static struct etimer et;
  // etimer_set(&et,CLOCK_SECOND*3);
  // int i=0;

  uploadFlag=false;
  
  if(join_mcast_group() == NULL) {

    PROCESS_EXIT();
  }
  
  PROCESS_PAUSE();

  set_global_address();
  
  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  client_conn_data=udp_new(NULL, UIP_HTONS(UDP_SERVER_DATA_PORT), NULL);
  
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);

  udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));

  n_id = uip_htons(node_id);

 task_schedule_set(&heart_ts,MUST_TASK,TASK_READY,TASK_PERIOD_DEFAULT,system_monitor_msg_send,NULL);
 task_schedule_set(&read_data_ts,MUST_TASK,TASK_READY,TASK_PERIOD_DEFAULT,send_read_meter_cmd,NULL);
  
  while(1) 
  {
    PROCESS_YIELD(); 
    if(ev == tcpip_event) 
    {

      tcpip_handler();
    }
    
    /*触发串口事件，读取串口助手的char数据，存在data里*/
    if (ev==serial_line_event_message2)
    {

    //  leds_toggle(LEDS_GREEN);   
      memset(ack_buffer,0,MAX_PAYLOAD_LEN);
      data_length =(int)((char *)data)[0];
     // data_length = (unsigned char)((char *)data)[0];
      ack_buffer[0] = global_type;
      ack_buffer[1] = CMD_HEATMETER;
      memcpy(ack_buffer + 2,(char *)data+1,data_length);
      // global_period = (random_rand() % (global_base_sec))+SEND_INTERVAL;
      // printf("data len:%d\n",data_length );
      uip_udp_packet_sendto(client_conn_data, ack_buffer, data_length+2,
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));

      // uploadFlag = true;
    }

        //   if(etimer_expired(&et))
        //  {
        //       char cmd[5]={0x10,0x5B,0xFE,0x59,0x16};
        //       for(i=0;i< 5;i++)
        //       {
        //           putchar((unsigned char )cmd[i]);
        //       }

        //   etimer_reset(&et);
        // }

   }

     PROCESS_END();
 }
/*---------------------------------------------------------------------------*/

// static void  error_msg_send()
// {
//    /* need define error code */

//   uip_udp_packet_sendto(client_conn_data, "h", 1, &server_ipaddr, UIP_HTONS(UDP_SERVER_DATA_PORT));
       
// }

static void  msg_handler(char *appdata,int appdata_length)
 {
 // #define T_INTERVAL  (120*CLOCK_SECOND/256)
    uint8_t type=(uint8_t)appdata[0];
  //  int return_val=0;
    static char buff[30];
    switch(type)
    {

      case CMD_SYSTEM_MONITOR:
          ctimer_set(&ct,random_delay(),system_monitor_msg_send,NULL);
      break;

      // case CMD_SYSTEM_MONITOR_1:
      //     ctimer_set(&ct,T_INTERVAL*n_id,system_monitor_msg1_send,NULL);
      // break;


      // case CMD_NETWORK_CONF:
      //      setting_network_configuration((uint8_t*)&appdata[1],appdata_length-1);  
      //                ctimer_set(&send_ct,T_INTERVAL*n_id,node_send,NULL); 
  
      // break;


      case CMD_NETWORK_CONF_1:
            setting_network_configuration1((uint8_t*)&appdata[1],appdata_length-1,&heart_ts);
                      // ctimer_set(&send_ct,T_INTERVAL*n_id,node_send,NULL); 

      break;


      case CMD_NETWORK_CONF_2:
           setting_network_configuration2((uint8_t*)&appdata[1],appdata_length-1);
                     // ctimer_set(&send_ct,T_INTERVAL*n_id,node_send,NULL); 

      break;


      case CMD_HEATMETER:

          memcpy(buff,appdata,appdata_length);
          buff[0] = appdata_length-1;
          // ctimer_set(&ct,T_INTERVAL*n_id,system_monitor_msg_send,NULL);
          ctimer_set(&send_ct,random_delay(),sendCmd,buff);
        //  sendCmd(&appdata[1],appdata_length-1);         
      break;
      
      
      case CMD_DATAREAD_CMD:

           //return_val=cmd_bytes_burn((uint8_t*)&appdata[1]);
           // //printf("return_val:%d\n",return_val );
       //    if(!return_val){
           restore_meter_cmd();
           // print_cmd_array();
          ctimer_set(&send_ct,random_delay(),send_read_meter_cmd,NULL);
        //  }           
          // task_schedule_set(&read_data_ts,MUST_TASK,TASK_READY,TASK_PERIOD_DEFAULT,send_read_meter_cmd,NULL);
           
      break;


      case CMD_REBOOT:
          ctimer_set(&ct,60 * CLOCK_SECOND,NodeReboot,NULL); 
          // ctimer_set(&send_ct,T_INTERVAL*n_id,node_send,NULL); 
 
      break;

      case CMD_RESET:
          ctimer_set(&ct,(60 * CLOCK_SECOND),NodeReset,NULL); 
         // /ctimer_set(&send_ct,T_INTERVAL*n_id,node_send,NULL); 
      break;

      case CMD_CHECK_WAKEUP:
        printf("wake check\n");
        leds_off(LEDS_GREEN);
        break;

      default:  
           //printf("msg handler type is %d\n",type );
          // sendCmd(&appdata[1],appdata_length-1);    
      break;

      }

      // error_msg_send()
 }




#if 0
static void
system_monitor_msg_send(void  * p)
{
  static uint8_t seqno;
  struct {
    //char heart;
    uint8_t seqno;
    uint8_t for_alignment;
    struct collect_view_data_msg msg;
  } msg;
  /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  uint16_t beacon_interval;
  rpl_parent_t *preferred_parent;
  linkaddr_t parent;
  rpl_dag_t *dag;

  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  memset(&msg, 0, sizeof(msg));
  seqno++;
  //msg.heart =HERAT_BODY;
  if(seqno == 0) {
    /* Wrap to 128 to identify restarts */
    seqno = 128;
  }
  msg.seqno = seqno;

  linkaddr_copy(&parent, &linkaddr_null);
  parent_etx = 0;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag != NULL) {
    preferred_parent = dag->preferred_parent;
    if(preferred_parent != NULL) {
      uip_ds6_nbr_t *nbr;
      nbr = uip_ds6_nbr_lookup(rpl_get_parent_ipaddr(preferred_parent));
      if(nbr != NULL) {
        /* Use parts of the IPv6 address as the parent address, in reversed byte order. */
        parent.u8[LINKADDR_SIZE - 1] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 2];
        parent.u8[LINKADDR_SIZE - 2] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 1];
        parent_etx = rpl_get_parent_rank((uip_lladdr_t *) uip_ds6_nbr_get_ll(nbr)) / 2;
      }
    }
    rtmetric = dag->rank;
    beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = uip_ds6_nbr_num();
  } else {
    rtmetric = 0;
    beacon_interval = 0;
    num_neighbors = 0;
  }

  /* num_neighbors = collect_neighbor_list_num(&tc.neighbor_list); */
  // collect_view_construct_message(&msg.msg, &parent,
  //                                parent_etx, rtmetric,
  //                                num_neighbors, beacon_interval);

  uip_udp_packet_sendto(client_conn, &msg, sizeof(msg),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

}
#endif