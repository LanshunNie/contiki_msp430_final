#ifndef RPL_REP_UTIL
#define RPL_REP_UTIL
#define bool int
#define true 1
#define false 0
#define error -1
bool reupload_by_acks(char * data,int length,int node_id);
#endif