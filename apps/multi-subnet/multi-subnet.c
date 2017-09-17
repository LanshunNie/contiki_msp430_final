/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: multi-subnet.c ,v 1.21 2017/03/26 21:30:15 
*/
/**
 * \file
 *        one large network to multi subnet.
 * \author
 *        @yang
 */

#include "contiki.h"
#include "multi-subnet.h"
#include "multi-subnet-conf.h"
#include "net/mac/frame802154.h"

#define DEBUG  0  //DEBUG_PRINT
#include "net/ip/uip-debug.h"

#if UIP_CONF_MULTI_SUBNET

static uint16_t own_subnet_panid = DEFAULT_SUBNET_PANID;
static uint16_t dio_receive_subnet_panid = DEFAULT_SUBNET_PANID;
static uint16_t link_receive_subnet_panid = DEFAULT_SUBNET_PANID;
/*---------------------------------------------------------------------------*/
void
set_own_subnet_panid(uint16_t panid){
  own_subnet_panid = panid;
}
/*---------------------------------------------------------------------------*/
uint16_t
get_own_subnet_panid(void){
  return own_subnet_panid;
}
/*---------------------------------------------------------------------------*/
uint16_t
get_default_subnet_panid(void){
  return DEFAULT_SUBNET_PANID;
}
/*---------------------------------------------------------------------------*/
void
set_dio_receive_subnet_panid(uint16_t panid){
  dio_receive_subnet_panid = panid;
}
/*---------------------------------------------------------------------------*/
uint16_t
get_dio_receive_subnet_panid(void){
  return dio_receive_subnet_panid;
}
/*---------------------------------------------------------------------------*/
void
set_link_receive_subnet_panid(uint16_t panid){
  link_receive_subnet_panid = panid;
}
/*---------------------------------------------------------------------------*/
uint16_t
get_link_receive_subnet_panid(void){
  return link_receive_subnet_panid;
}
/*---------------------------------------------------------------------------*/
uint8_t
subnet_panid_limit(uint16_t dest_pid)
{
  if(dest_pid != own_subnet_panid && dest_pid != 0xFFFF &&
    own_subnet_panid != 0xFFFF) {
    /* Packet to another PAN */
    PRINTF("Packet to another pan %u\n", dest_pid);
    return 0;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
#endif


