#include "util.h"

bool reupload_by_acks(char * data,int length,int node_id){
	if(node_id > (length * 8)){
		return error;
	}
	node_id--;
	int byte_offset = node_id >> 3;
	int bit_offset = node_id & 0x07;
	bool ret = 0x01 << bit_offset;
	if(!(ret & (*(data + byte_offset)))){
		return false;
	}
	return true;
}