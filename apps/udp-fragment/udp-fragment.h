/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: udp-fragment.c ,v 1.21 2016/7/15 
 */

/**
 * \file
 *         A udp packet fragment mechanism
 * \author
 *         @yang
 */

#ifndef UDP_FRAGMENT_H
#define UDP_FRAGMENT_H

#include "net/ip/uip.h"
#include "contiki-conf.h"

#if UIP_CONF_IPV6_ORPL_BITMAP
#include "net/orpl-bitmap/orpl-bitmap.h"
#endif

#define UNFRAGMENT_FLAG  0x00
#define FRAGMENT_FLAG    0xC0

#define UNFRAGMENT_MAX_LEN  68   //bytes
#define FRAGMENT_MAX_LEN  (UNFRAGMENT_MAX_LEN-3)
#define SEND_FAIL_SUB_LEN  4

#ifdef  UIP_CONF_BUFFER_SIZE
#define DATABUF_SIZE     UIP_CONF_BUFFER_SIZE
#else
#define DATABUF_SIZE     200
#endif

//#define UDP_CONF_REASSEMBLY  1 //0

#if UDP_CONF_REASSEMBLY
#define FRAG_BIT_NUM   8      //8 de beishu
#define REASSMBLY_PACKET_NUM   20   //must less than 0xfe
#define UDP_REASSMBLY_MAXAGE    (60 * CLOCK_SECOND)
#endif

struct udp_frag_info_t{
    struct uip_udp_conn c;
    char databuf[DATABUF_SIZE]; 
    int data_len;
    uint16_t next_offset;
    uint8_t  frag_seq;
    uint8_t  unclean_buf_flag;
};

#if UDP_CONF_REASSEMBLY
typedef void (* udp_reassembly_callback_t)(const void *data, int len,uint16_t src_addr);

struct udp_reassem_info_t{
    char databuf[DATABUF_SIZE]; 
    int data_len;
    uint16_t src_addr;
    uint8_t  frag_seq;
    uint8_t  frag_total_num;   
    uint8_t  frag_order_set[FRAG_BIT_NUM/8];
    struct ctimer ct;         //set reassem  timer 
    uint8_t start_ct_flag;
    udp_reassembly_callback_t reassembly_callback;
};

struct udp_reassem_info_h{
	uint16_t src_addr;
    uint8_t  frag_seq;
	uint8_t frag_order;
	uint8_t remain_frag_flag;
    uint16_t frag_offset;
    udp_reassembly_callback_t reassembly_callback;
};

#endif

void udp_fragment_init();

void udp_fragment_packet_send(struct uip_udp_conn *c, const void *data, int len);

void udp_fragment_send_callback(void);

#if UDP_CONF_REASSEMBLY

uint8_t udp_reassembly_parse(uip_ipaddr_t * src_ipaddr,const void *data,int len,udp_reassembly_callback_t reassembly_callback);
#endif

#endif /* UDP_FRAGMENT_H */