/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
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
 * $Id: node-id.c,v 1.2 2010/08/26 22:08:11 nifi Exp $
 */

/**
 * \file
 *         Utility to store a node id in the infomem A
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Fredrik Osterlind <fredrik@thingsquare.com>
 */

#include "node-id.h"
#include "contiki-conf.h"
#include "dev/flash.h"

#include <stdio.h>
#include <string.h>
#include "sys/energest.h"
#include "sys/clock.h"


#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define ADDR_INFOMEM_A     0x1980        //node-id
#define ADDR_INFOMEM_B     0x1900        // normal byte rfchannel
#define ADDR_INFOMEM_C     0x1880        // cmd
#define ADDR_INFOMEM_D     0x1800        // restart  count 

unsigned short node_id = 0;
unsigned char node_mac[8];

uint8_t magic_id[] = {0xab, 0xcd};
/*---------------------------------------------------------------------------*/
void
node_id_restore(void)
{
  uint8_t *infomem;
  infomem = (uint8_t *) ADDR_INFOMEM_A; /* protected infomem A */
  uint8_t i;

  if(infomem[0] == magic_id[0] && infomem[1] == magic_id[1]) {
    for(i = 0; i < 8; i++) {
      node_mac[i] = infomem[2 + i];
    }

  } else {
    /* default address */
    node_mac[0] = 0x02;
    node_mac[1] = 0x12;
    node_mac[2] = 0x74;
    node_mac[3] = 0x00;
    node_mac[4] = 0x00;
    node_mac[5] = 0x01;
    node_mac[6] = 0x02;
    node_mac[7] = 0x03;
    printf("node restore id ERROR\n");

  }
  memcpy(&node_id, &node_mac[6], 2);
}
/*---------------------------------------------------------------------------*/
void
node_id_burn(unsigned short id)
{
  unsigned char data[10];
  unsigned short val;
 
  data[0] = magic_id[0];
  data[1] = magic_id[1];
  data[2] = 0x02;
  data[3] = 0x12;
  data[4] = 0x74;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x01;

  printf("burn nodeid :%x\n",id);
  
  flash_setup();
  flash_clear((unsigned long)(ADDR_INFOMEM_A));
  memcpy(&val, &data[0], 2);
  flash_write((unsigned long)(ADDR_INFOMEM_A), val);
  memcpy(&val, &data[2], 2);
  flash_write((unsigned long)(ADDR_INFOMEM_A + 2), val);
  memcpy(&val, &data[4], 2);
  flash_write((unsigned long)(ADDR_INFOMEM_A + 4), val);
  memcpy(&val, &data[6], 2);
  flash_write((unsigned long)(ADDR_INFOMEM_A + 6), val);

  id= (id>>8)+((id&0x00ff)<<8);  //add
  flash_write((unsigned long)(ADDR_INFOMEM_A + 8), id);
  flash_done();
}
/*---------------------------------------------------------------------------*/
/*
*  for normal btye
*  
*/

void normalbyte_rfchannel_burn(uint16_t normalbyte , uint8_t rfchannel)
{

    printf("write normalbyte-- %x to flash\n", normalbyte);
    printf("write channel byte-- %x to flash\n", rfchannel);
    flash_setup();
    flash_clear((unsigned long)(ADDR_INFOMEM_B));
    flash_write((unsigned long)(ADDR_INFOMEM_B),normalbyte);
    flash_write((unsigned long)(ADDR_INFOMEM_B + 2),rfchannel);
    flash_done();
}
void normalbyte_rfchannel_restore(void)
{
    uint8_t *infomem;

    infomem = (uint8_t *) ADDR_INFOMEM_B; /* protected infomem A */
    normal_byte = infomem[0]|(infomem[1]);
    channel_byte= infomem[2]|(infomem[3]);

    set_init_flag(normal_byte);
    printf("normalbyte restore :%x\n",normal_byte);
    printf("channel_byte restore %x\n",channel_byte);
}

/*
*  for meter reading data command burn
*  
*/

void restore_meter_cmd(void)
{
     int i = 0;
     uint16_t length=0;
     uint8_t *infomem;

     infomem =(uint8_t*)ADDR_INFOMEM_C;

     length =(infomem[1]<<8)|infomem[0];
      PRINTF("length:0x%x\n", length);

   if( length > (uint16_t)LENGTH_OF_METER_READ_CMD )  
     {
        PRINTF("wrong length ,please write cmd in flash \n");
        cmd_read_meter[0] =0xff;
        cmd_read_meter[1] =0xff;
      return ;
     }
     cmd_read_meter[0] =infomem[1];
     cmd_read_meter[1] =infomem[0];

   for (; i < length; i++)
   {
       cmd_read_meter[i+2] = infomem[i+2];   
       PRINTF("data %d :%2x\n",i,  cmd_read_meter[i+2] );
   }

}

void print_cmd_array(void)
{
  int i = 0;
  uint16_t length=0;
  // uint8_t *infomem;
  //    infomem =(uint8_t*)ADDR_INFOMEM_METER_CMD_START;
  // length =(infomem[1]<<8)|infomem[0];
  length =(cmd_read_meter[0]<<8)|cmd_read_meter[1];
 
  printf("cmd array (%x):",length);
  if( length > (uint16_t)LENGTH_OF_METER_READ_CMD )  
  {
        PRINTF("wrong length ,no cmd bytes in flash \n");
      return ;
  }
        for (; i <cmd_read_meter[1]; ++i)
        {
          printf("%02x",cmd_read_meter[i+2]);
        }
        printf("\n");
}

int cmd_bytes_burn(uint8_t *array)
{
    int i=0,j=2;
    uint16_t length=array[0];

    uint16_t temp=0;
    
    if(length> LENGTH_OF_METER_READ_CMD)
    {
      return 1;
    }
    flash_setup();
    flash_clear((unsigned long)(ADDR_INFOMEM_C));
  
    flash_write((unsigned long)(ADDR_INFOMEM_C),length);

    for ( ; i < length; i++,j+=2)
    {

        temp=array[i+1];
 
        ++i;
        if(i<length)
        {
          temp|=(array[i+1]<<8);
        }
      PRINTF("write temp: %x\n",temp );
      flash_write( (unsigned long) (ADDR_INFOMEM_C  + j ) , temp);       
    }
 
    flash_done();
    return 0;
}

/*void energy_bytes_init_burn(void)
{
    flash_setup();
    flash_clear((unsigned long)(ADDR_INFOMEM_ENERGY_START));

   energy_bytes_burn(ADDR_INFOMEM_CPU     ,0);
   energy_bytes_burn(ADDR_INFOMEM_LPM     ,0);
   energy_bytes_burn(ADDR_INFOMEM_TRANSMIT,0);
   energy_bytes_burn(ADDR_INFOMEM_LISTEN  ,0);

    flash_done();

}
void store_energy(uint64_t cpu,uint64_t lpm,uint64_t trans,uint64_t listen)
{
   flash_setup();
   flash_clear((unsigned long)(ADDR_INFOMEM_ENERGY_START));

   energy_bytes_burn(ADDR_INFOMEM_CPU     ,cpu);
   energy_bytes_burn(ADDR_INFOMEM_LPM     ,lpm);
   energy_bytes_burn(ADDR_INFOMEM_TRANSMIT,trans);
   energy_bytes_burn(ADDR_INFOMEM_LISTEN  ,listen);
   
   flash_done();
}


void get_energy(uint64_t *cpu,uint64_t *lpm,uint64_t *transmit,uint64_t *listen)
{    
      bytes_read(ADDR_INFOMEM_CPU,cpu);
      bytes_read(ADDR_INFOMEM_LPM,lpm);
      bytes_read(ADDR_INFOMEM_TRANSMIT,transmit);
      bytes_read(ADDR_INFOMEM_LISTEN,listen);
      PRINTF("get energy bytes : %llx :%llx :%llx : %llx\n",*cpu,*lpm,*transmit,*listen );
}

void energy_bytes_restore()
{

   get_energy(&(energest_total_time[ENERGEST_TYPE_CPU].current),
              &(energest_total_time[ENERGEST_TYPE_LPM].current),
              &(energest_total_time[ENERGEST_TYPE_TRANSMIT].current),
              &(energest_total_time[ENERGEST_TYPE_LISTEN].current)
              );
}
*/


/*
*
*   for channel
*/

void restart_count_byte_burn(unsigned short val)
{
    PRINTF("write reatartcount byte-- %x to flash\n", val);
    flash_setup();
    flash_clear((unsigned long)(ADDR_INFOMEM_D));
    flash_write((unsigned long)(ADDR_INFOMEM_D),val);
    flash_done();
}
void restart_count_byte_restore(void)
{
    uint8_t *infomem;

    infomem = (uint8_t *) ADDR_INFOMEM_D; /* protected infomem A */
    restart_count=(infomem[1]<<8)|infomem[0];
  
    PRINTF("restart count restore %x\n",restart_count);
 
}
