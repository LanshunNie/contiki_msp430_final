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


#include "contiki-net.h"
#include "udp-fragment.h"
#include "net/ipv6/multicast/uip-mcast6.h"
 
#include <string.h>
#include <stdio.h>

#define DEBUG  0   //DEBUG_NONE
#include "net/ip/uip-debug.h"

extern uint16_t uip_slen;

#define UDP_FRAG_SEND_UNIT_DELAY   (NETSTACK_RDC.channel_check_interval())
#define UDP_FRAG_SEND_SPREAD   2

static uint8_t  unfragment_send_max_len =  UNFRAGMENT_MAX_LEN;
static uint8_t  fragment_send_max_len = FRAGMENT_MAX_LEN;
static uint8_t  next_frag_seqno;
static struct   ctimer udp_frag_send_timer;

static struct udp_frag_info_t udp_frag_data_info;

#if UDP_CONF_REASSEMBLY
static struct udp_reassem_info_t udp_reassem_data_info[REASSMBLY_PACKET_NUM];
static struct udp_reassem_info_h udp_reassem_hinfo;
#endif
/*----------------------------------------------------------------------------------*/
#if 0
static void 
update_next_frag_seqno(){
   next_frag_seqno++;
}
#endif
/*----------------------------------------------------------------------------------*/
static uint16_t
random_delay(){
   return (UDP_FRAG_SEND_UNIT_DELAY * (1 + (random_rand() % UDP_FRAG_SEND_SPREAD)));  
}
/*----------------------------------------------------------------------------------*/
static void
udp_fragment_frag_send(){
   
   uint8_t remain_frag_flag = 0;
   uint8_t frag_order_num = 0;
   uint8_t frag_send_datalen = 0;
   char buf[10];
   int pos =0;

   /* hava data to send or to deal if uip_len >0 */
   if(uip_len > 0){  
     PRINTF("udp frag send fail,because collision\n ");

     ctimer_set(&udp_frag_send_timer, random_delay(),
             udp_fragment_frag_send,NULL);
     return;
   }
   
   if(udp_frag_data_info.next_offset + fragment_send_max_len < udp_frag_data_info.data_len){
      remain_frag_flag = 1;
   }
   
    frag_order_num = udp_frag_data_info.next_offset/fragment_send_max_len;
    
    PRINTF("frag_order_num:%u\n",frag_order_num);
    PRINTF("frag_seq:%u\n",udp_frag_data_info.frag_seq);

    uip_udp_conn = &udp_frag_data_info.c;

    PRINTF("dest ip addr:");
    PRINT6ADDR(&uip_udp_conn->ripaddr);
    PRINTF("\n");

#if ORPL_BITMAP_DUPLICATE_DETECTION 
    uint16_t current_seqno=orpl_bitmap_get_current_seqno();
  //  printf("send seqno:%02x\n",current_seqno);
    memcpy(buf+pos,&current_seqno, sizeof(current_seqno));
  //   memcpy(&buf[0],&current_seqno, sizeof(current_seqno));
    pos+=2;  
#endif /* ORPL_BITMAP_DUPLICATE_DETECTION */
      
    buf[pos++] = FRAGMENT_FLAG| (frag_order_num<<1) |remain_frag_flag;      //

    buf[pos++] = udp_frag_data_info.frag_seq;
    memcpy(buf+pos,&udp_frag_data_info.next_offset, sizeof(udp_frag_data_info.next_offset));
    pos += sizeof(udp_frag_data_info.next_offset);
    
    frag_send_datalen = udp_frag_data_info.data_len - udp_frag_data_info.next_offset > fragment_send_max_len?
          fragment_send_max_len: udp_frag_data_info.data_len - udp_frag_data_info.next_offset;
    
    uip_slen = frag_send_datalen + pos;

    memmove(&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN],buf,pos);
    memmove(&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN + pos],
    	&udp_frag_data_info.databuf[udp_frag_data_info.next_offset],frag_send_datalen);    

    uip_process(UIP_UDP_SEND_CONN);

#if UIP_CONF_IPV6_MULTICAST
  /* Let the multicast engine process the datagram before we send it */
   if(uip_is_addr_mcast_routable(&uip_udp_conn->ripaddr)) {
      UIP_MCAST6.out();
    }
#endif /* UIP_IPV6_MULTICAST */

#if NETSTACK_CONF_WITH_IPV6
    tcpip_ipv6_output();
#else
    if(uip_len > 0) {
      tcpip_output();
    }
#endif

   udp_frag_data_info.next_offset = udp_frag_data_info.next_offset + fragment_send_max_len;

   if(udp_frag_data_info.next_offset >= udp_frag_data_info.data_len){
      PRINTF("frag send over \n");
      memset(&udp_frag_data_info, 0, sizeof(udp_frag_data_info));
      ctimer_stop(&udp_frag_send_timer);
   }else{      
      PRINTF("delay  send  next frag \n");
      ctimer_set(&udp_frag_send_timer, random_delay(),
             udp_fragment_frag_send,NULL);
   } 
}
/*----------------------------------------------------------------------------------*/
static void 
udp_fragment_unfrag_send(struct uip_udp_conn *c, const void *data, int len){
    
    uip_udp_conn = c;
    char buf[10];
    int pos =0;

#if ORPL_BITMAP_DUPLICATE_DETECTION 
    uint16_t current_seqno=orpl_bitmap_get_current_seqno();
  //  printf("send seqno:%02x\n",current_seqno);
    memcpy(buf+pos,&current_seqno, sizeof(current_seqno));
  //   memcpy(&buf[0],&current_seqno, sizeof(current_seqno));
    pos+=2;  
#endif /* ORPL_BITMAP_DUPLICATE_DETECTION */
      
    buf[pos++] = UNFRAGMENT_FLAG;   
    uip_slen = len + pos;

    memmove(&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN],buf,pos);
    memmove(&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN + pos],data,len);    

    uip_process(UIP_UDP_SEND_CONN);

#if UIP_CONF_IPV6_MULTICAST
  /* Let the multicast engine process the datagram before we send it */
   if(uip_is_addr_mcast_routable(&uip_udp_conn->ripaddr)) {
      UIP_MCAST6.out();
    }
#endif /* UIP_IPV6_MULTICAST */

#if NETSTACK_CONF_WITH_IPV6
    tcpip_ipv6_output();
#else
    if(uip_len > 0) {
      tcpip_output();
    }
#endif
}
/*----------------------------------------------------------------------------------*/
void 
udp_fragment_packet_send(struct uip_udp_conn *c, const void *data, int len){
  
  uint8_t mcast_send_more_len = 0;

  if(len > DATABUF_SIZE){
  	PRINTF("udp frag packet is too large\n");
  	return;
  }
  
  if(uip_is_addr_mcast(&c->ripaddr)){

#if UIP_CONF_IPV6_ORPL_BITMAP
    mcast_send_more_len += 3;
#endif	
  	mcast_send_more_len += 6;
  }

  if(len >unfragment_send_max_len + mcast_send_more_len){
    if(udp_frag_data_info.unclean_buf_flag){
      /*  clean old ,make space for new */
       memset(&udp_frag_data_info, 0, sizeof(udp_frag_data_info));
       ctimer_stop(&udp_frag_send_timer);
       PRINTF("udp have old frag unsent, so clean old\n");   
    }
       memmove(&udp_frag_data_info.databuf,data,len);
       
       memcpy(&udp_frag_data_info.c,c, sizeof(struct uip_udp_conn));

       udp_frag_data_info.data_len = len;
       udp_frag_data_info.frag_seq = next_frag_seqno;
       udp_frag_data_info.unclean_buf_flag = 1;
       udp_frag_data_info.next_offset = 0;

       next_frag_seqno ++;
       
       PRINTF("udp frag send\n");
       udp_fragment_frag_send();
    
  }else{
    PRINTF("unfrag send\n");
    udp_fragment_unfrag_send(c,data,len);
  }
}
/*----------------------------------------------------------------------------------*/
void 
udp_fragment_send_callback(){
  PRINTF("udp fragment packet is big\n");

  unfragment_send_max_len = unfragment_send_max_len -SEND_FAIL_SUB_LEN;
  fragment_send_max_len = fragment_send_max_len -SEND_FAIL_SUB_LEN; 
  if(udp_frag_data_info.unclean_buf_flag){
  	 udp_frag_data_info.next_offset = 0;
  	 udp_fragment_frag_send();
  }
}
/*----------------------------------------------------------------------------------*/
#if UDP_CONF_REASSEMBLY

/* Get a bit in a frag order set */
static uint8_t
frag_order_set_get_bit(struct udp_reassem_info_t *udp_reas_info,uint8_t frag_order) {
  return (udp_reas_info->frag_order_set[frag_order/8] & (1 << (frag_order%8))) != 0;
}
/*----------------------------------------------------------------------------------*/
/* Inserts a frag order flag in the frag order set */
static void
frag_order_set_insert(struct udp_reassem_info_t *udp_reas_info,uint8_t frag_order){
    udp_reas_info->frag_order_set[frag_order/8] |= (1 << (frag_order%8));
} 
/*----------------------------------------------------------------------------------*/
/* Checks if frag order set contains a frag */
static uint8_t
frag_order_set_contains_frag(struct udp_reassem_info_t *udp_reas_info,uint8_t frag_order)
{
  uint8_t contains = 1;
  /* For each frag order, check a bit in the frag order set */
  if(frag_order_set_get_bit(udp_reas_info,frag_order) == 0) {
    /* If one bucket is empty, then the frag isn't included in the frag set */
     contains = 0;
  } 
  return contains;
}
/*----------------------------------------------------------------------------------*/
/* Returns the number of bits set in the frag order set */
static int
frag_order_set_count_bits(struct udp_reassem_info_t *udp_reas_info)
{
  int i;
    int cnt = 0;
    for(i=0; i<FRAG_BIT_NUM; i++) {
      if(frag_order_set_get_bit(udp_reas_info,i)) {
        cnt++;
      }
    }
  return cnt;
}
/*----------------------------------------------------------------------------------*/
static void
udp_reassembly_init(){
   memset(&udp_reassem_data_info, 0, sizeof(udp_reassem_data_info));
   memset(&udp_reassem_hinfo , 0, sizeof(udp_reassem_hinfo));
}
/*----------------------------------------------------------------------------------*/
static void
udp_reassembly_packetbuf_reset(void *udp_reas_info){
   memset((struct udp_reassem_info_t *)udp_reas_info, 0, sizeof(struct udp_reassem_info_t));
}
/*----------------------------------------------------------------------------------*/
static int
udp_reassembly_packetbuf_freespace(){
  int i;
  for(i= 0;i<REASSMBLY_PACKET_NUM;i++){
    if(udp_reassem_data_info[i].data_len == 0) 
   		return i+1;	
  }
  return 0;
}
/*----------------------------------------------------------------------------------*/
/*
  if duplicate return 0, else return reassembly index +1;
  if new packet frag ,return 0xff;
*/

static int
udp_reassembly_duplicate_detect( ){
  int i;
  uint8_t contain_flag =0;

  for(i= 0;i<REASSMBLY_PACKET_NUM;i++){
    if(udp_reassem_data_info[i].data_len == 0)
    	continue;
   	if(udp_reassem_data_info[i].src_addr == udp_reassem_hinfo.src_addr){
        if(udp_reassem_data_info[i].frag_seq != udp_reassem_hinfo.frag_seq)
           return (i+1);  
   		contain_flag = frag_order_set_contains_frag(&udp_reassem_data_info[i],udp_reassem_hinfo.frag_order);
   		if(contain_flag)
   		   return 0;
   		return (i+1);
   	}
  }
  return 0xff;
}
/*----------------------------------------------------------------------------------*/
static void
udp_reassembly_reassemble_newfrag(int index,const void *data,int len){
  
  int reassemble_frag_num =0;
  if(udp_reassem_hinfo.remain_frag_flag == 0){
   	 udp_reassem_data_info[index].frag_total_num = udp_reassem_hinfo.frag_order +1;
  }
  frag_order_set_insert(&udp_reassem_data_info[index],udp_reassem_hinfo.frag_order);
  udp_reassem_data_info[index].data_len += len;
  memcpy(&(udp_reassem_data_info[index].databuf[udp_reassem_hinfo.frag_offset]),data,len);
  if(udp_reassem_data_info[index].frag_total_num > 0){
     reassemble_frag_num = frag_order_set_count_bits(&udp_reassem_data_info[index]);
     if(udp_reassem_data_info[index].frag_total_num == reassemble_frag_num){
       //reassemble over ,do callback;
     	
     	if(udp_reassem_data_info[index].reassembly_callback){ 
     	  PRINTF("reassemble over ,do callback\n");
         udp_reassem_data_info[index].reassembly_callback(udp_reassem_data_info[index].databuf,
     	   	                           udp_reassem_data_info[index].data_len,udp_reassem_data_info[index].src_addr);
     	   ctimer_stop(&udp_reassem_data_info[index].ct);
           udp_reassembly_packetbuf_reset(&udp_reassem_data_info[index]);
     	}
     }
  }
}
/*----------------------------------------------------------------------------------*/
static void
udp_reassembly_reassemble_newpacket(int index,const void *data,int len){
   
  udp_reassem_data_info[index].src_addr = udp_reassem_hinfo.src_addr;
  udp_reassem_data_info[index].frag_seq = udp_reassem_hinfo.frag_seq;

  udp_reassem_data_info[index].start_ct_flag =1;
  ctimer_set(&udp_reassem_data_info[index].ct, UDP_REASSMBLY_MAXAGE,
             udp_reassembly_packetbuf_reset,&udp_reassem_data_info[index]);

  udp_reassem_data_info[index].reassembly_callback=udp_reassem_hinfo.reassembly_callback;

  udp_reassembly_reassemble_newfrag(index,data,len);
}

/*----------------------------------------------------------------------------------*/
/*
  if frag duplicate return 0,drop;
  if unfrag packet ,return 1, app do it self;
  if new frag packet ,return 2, app set callback fun to receive reassembly packet.
*/

uint8_t
udp_reassembly_parse(uip_ipaddr_t * src_ipaddr,const void *data,int len,udp_reassembly_callback_t reassembly_callback){
 
   int pos =0;
   uint8_t frag_type,frag_flag;
   uint8_t non_dup_flag,freespace_flag;

   uint16_t src_addr = ((src_ipaddr->u8[14])<<8) + (src_ipaddr->u8[15]);
   udp_reassem_hinfo.src_addr = src_addr;
   udp_reassem_hinfo.reassembly_callback = reassembly_callback;

#if ORPL_BITMAP_DUPLICATE_DETECTION 
   pos+=2;  
#endif /* ORPL_BITMAP_DUPLICATE_DETECTION */
   
   if(len <= pos)
      return 0;

   memcpy(&frag_type, (uint8_t *)(data+pos), sizeof(frag_type));  
   pos += sizeof(frag_type);

   frag_flag = (frag_type>>6)&3;
   if(frag_flag){
   	  udp_reassem_hinfo.frag_order = ((frag_type & (~FRAGMENT_FLAG))>>1);
   	  udp_reassem_hinfo.remain_frag_flag = frag_type & 1;

   	  memcpy(&udp_reassem_hinfo.frag_seq, (uint8_t *)(data+pos), sizeof(udp_reassem_hinfo.frag_seq));  
      pos += sizeof(udp_reassem_hinfo.frag_seq);
      memcpy(&udp_reassem_hinfo.frag_offset, (uint8_t *)(data+pos), sizeof(udp_reassem_hinfo.frag_offset));  
      pos += sizeof(udp_reassem_hinfo.frag_offset);
     
      non_dup_flag = udp_reassembly_duplicate_detect();
      if(!non_dup_flag)
      	return 0;
      if(non_dup_flag == 0xff){
          freespace_flag = udp_reassembly_packetbuf_freespace();
          if(!freespace_flag){
          	PRINTF("drop new packet frag ,because no space\n");
          	return 0;
          }
         // udp_reassembly_packetbuf_reset(&udp_reassem_data_info[freespace_flag-1]);
          udp_reassembly_reassemble_newpacket(freespace_flag-1,data+pos,len-pos);
      }else{
      	  if(udp_reassem_data_info[non_dup_flag-1].frag_seq != udp_reassem_hinfo.frag_seq){
             PRINTF("drop old frag,eassembly new \n");
             ctimer_stop(&udp_reassem_data_info[non_dup_flag-1].ct);
             udp_reassembly_packetbuf_reset(&udp_reassem_data_info[non_dup_flag-1]);
             udp_reassembly_reassemble_newpacket(non_dup_flag-1,data+pos,len-pos);
      	  }else{
             PRINTF("reassembly new frag\n");
             udp_reassembly_reassemble_newfrag(non_dup_flag-1,data+pos,len-pos);
      	  }
      }  
   }
   return frag_flag+1;
}

#endif
/*----------------------------------------------------------------------------------*/
void 
udp_fragment_init(){
  next_frag_seqno = random_rand();
  memset(&udp_frag_data_info, 0, sizeof(udp_frag_data_info));
#if UDP_CONF_REASSEMBLY
  udp_reassembly_init();
#endif  
}
/*----------------------------------------------------------------------------------*/


