#ifndef __NODE_FUNCTION_H__
#define __NODE_FUNCTION_H__
#include "sys/clock.h"

#if CONTIKI_CONF_NETSYNCH

#endif 
#include "netsynch.h"
#include "task-schedule.h"

#include "sys/energest.h"
#include "node-id.h"
#include "net/rpl/rpl-conf.h"

enum 
{
	RETURN_ERROR,
	RETURN_OK
};


/*====================================================================================================*/
// # 系统监测类指令  参数上报 18+3+2+24+1 =48

#define CMD_SYSTEM_MONITOR      0x00  

//#define INDEX_SCHEDULE  		    0 //18	    //   调度信息   18byte schedule 
#define INDEX_TIME                  0//3       //   时间同步   3 byte time
#define INDEX_TOPO  				3//2	    //   网络拓扑            2 byte    parent id
#define INDEX_ENERGYCOST  		    5//24	    //   能耗              4*6 byte   record 200+years
#define INDEX_ADCVOLTAGE  		    29//2	    //   采样电压            1 byte    true or false 

#define INDEX_BEACON_INTERVAL		31//2
#define INDEX_NUM_NEIGHBORS			33//2
#define INDEX_RTMETRIC				35//2

#define INDEX_TIME_DIFF				37//1		//  时间差
#define INDEX_RESTART_COUNT                                       38//1

#define INDEX_CYCLETIME 			39//2
#define INDEX_CYCLETIME_DIRECTION   41//1
#define INDEX_CURRENT_BUDGET        42//4

#define INDEX_NODE_BEHAVIOUR        46//10 

#define INDEX_END                   56

#define SYSTEM_MONITOR_MSG_LENGTH (INDEX_END)

#define CMD_SYSTEM_MONITOR_1    0x01    //           长度1byte 
#define INDEX_PANID               0       //PAN id     长度1byte 
#define INDEX_CHANNEL             1       //信道        长度1byte 
#define INDEX_CCATHR              2       //CCA阈值     长度1byte 
#define INDEX_TRANSMIT_POWER      3       //发射功率     长度1byte  
#define INDEX_ACTIVE_CCA_CHECKT_RATE     4       //CCA检查频率  长度1byte 
#define INDEX_INACTIVE_CCA_CHECKT_INTERVAL  5    //CCA检查interval  长度1byte    
#define RPL_CONF_DIO_INTERVAL_MIN_CONF	    6
#define RPL_CONF_DIO_INTERVAL_DOUBLINGS_CONF 7
#define SYSTEM_MONITOR_MSG_LENGTH1 (RPL_CONF_DIO_INTERVAL_DOUBLINGS_CONF+1) 

typedef uint8_t system_monitor_msg[SYSTEM_MONITOR_MSG_LENGTH];
typedef uint8_t system_monitor_msg1[SYSTEM_MONITOR_MSG_LENGTH1];

/*====================================================================================================*/


// # 网络配置指令 参数下发

#define CMD_NETWORK_CONF        0x40 // [0x40 5byte] panid　+channel　+cca阈值　+发射功率 +CCA检查周期
#define CMD_NETWORK_CONF_1      0x41 // [0x41 1byte]   配置网络拓扑 能耗数据 采样电压 上报周期   // #define TOPO_ENERGYCOST_ADCV_PERIOD     
#define CMD_NETWORK_CONF_2      0x42 //

#define PANID 					0    //    配置PANID
#define RFCHANNEL 				1    //    配置channel
#define CCATHR 					2    //    配置CCA阈值
#define RFTRANSMITPOWER 		3    //    配置发射功率
#define RFCHANNEL_INACTIVE_CHECK_RATE    4    //	   配置CCA检查周期
#define RFCHANNEL_ACTIVE_CHECK_INTERVAL    5    //	   配置CCA检查Interval
#define RPL_CONF_DIO_INTERVAL_MIN_CONF		6  //	   配置DIO MIN PERIOD	
#define RPL_CONF_DIO_INTERVAL_DOUBLINGS_CONF	7  //  配置DIO DOUBLINGS	


#define NETWORK_CONF_LENGTH     (RPL_CONF_DIO_INTERVAL_DOUBLINGS_CONF+1)
#define NETWORK_CONF_LENGTH1    1
#define NETWORK_CONF_LENGTH2    18


/*====================================================================================================*/


// # 业务功能指令 

#define CMD_HEATMETER 			    0x82    //    给表的指令
#define CMD_GET_NODE_METERID	    0x81    //    获取表ID和节点ID
#define CMD_DATAREAD_CMD	      0x80    //    获取表ID和节点ID

/*====================================================================================================*/

// # 系统功能指令 

#define CMD_REBOOT      			 0xC0     //    点重启
#define CMD_RESET 					 0xC1     //    复出厂设置

/*====================================================================================================*/

/*system monitoring function*/
void get_system_monitor_msg(uint8_t array[],int length);
void system_monitor_msg_send(void *p);
void get_system_monitor_msg1(uint8_t array[],int length);

/*Network Configuration function*/
void setting_network_configuration(uint8_t array[],int length);
void setting_network_configuration1(uint8_t array[],int length,struct task_schedule *ts);
void setting_network_configuration2(uint8_t array[],int length);
/*Main Function */

// void heatMeterCommandEXE();
// void get_NodeID_MeterID();
int MeterCommandBurn();

/*reset or initialization instruction*/
void NodeReboot(void *p);
void NodeReset(void *p);

//zhangwei set changed for load balance
uint8_t getNetDataTaskPeriod(void);
// void msg_handler(char *appdata,int appdata_length);

// void system_monitor_msg_send(void  * p);

// void system_monitor_msg1_send(void  * p);


#endif