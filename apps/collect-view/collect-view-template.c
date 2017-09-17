#include "collect-view.h"
//#include "sys/clock.h"

enum {
  SENSOR0,
  SENSOR1,
  SENSOR2,
  SENSOR3,
  SENSOR4,  
  SENSOR5,
  SENSOR6,  
  SENSOR7,
  TIME1,
  TIME2,
};
//soft_time timenow;

/*---------------------------------------------------------------------------*/
void
collect_view_arch_read_sensors(struct collect_view_data_msg *msg)
{

  //get_timenow(&timenow);

  msg->sensors[SENSOR1] = 0;
  msg->sensors[SENSOR2] = 0;
  msg->sensors[SENSOR0]=0;
  msg->sensors[SENSOR1]=0;
  msg->sensors[SENSOR2]=0;
  msg->sensors[SENSOR3]=0;
  msg->sensors[SENSOR4]=0;
  msg->sensors[SENSOR5]=0;
  msg->sensors[SENSOR6]=0;  
  msg->sensors[SENSOR7]=0;

//  msg->sensors[TIME1]=timenow.hour;
//  msg->sensors[TIME2]=timenow.minute<<8|timenow.sec;
}
/*---------------------------------------------------------------------------*/




