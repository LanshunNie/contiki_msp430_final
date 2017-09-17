/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 */

 /*
    modified  by Guoxuenan HITCPS gxnhit@gmail.com
    note:
    
    add:
    PROCESS(serial_line_process2,"Serial driver2");
    process_event_t serial_line_event_message2;
    serial_line_input_byte2(unsigned char c)

    if size of message is bigger than 128byte , size of array rxbuf_data must be changed and  must be a power of two.for example if 
    your message size is 129byte you must change the size of array rxbuf_data to BUFSIZE+128.

 */
#include "dev/serial-line.h"
#include <string.h> /* for memcpy() */

#include "lib/ringbuf.h"
//#include "dev/leds.h"

#ifdef SERIAL_LINE_CONF_BUFSIZE
#define BUFSIZE SERIAL_LINE_CONF_BUFSIZE
#else /* SERIAL_LINE_CONF_BUFSIZE */
#define BUFSIZE 128
#endif /* SERIAL_LINE_CONF_BUFSIZE */

#if (BUFSIZE & (BUFSIZE - 1)) != 0
#error SERIAL_LINE_CONF_BUFSIZE must be a power of two (i.e., 1, 2, 4, 8, 16, 32, 64, ...).
#error Change SERIAL_LINE_CONF_BUFSIZE in contiki-conf.h.
#endif

#define IGNORE_CHAR(c) (c == 0x0d)
#define END 0x0a


PROCESS(serial_line_process, "Serial driver");
process_event_t serial_line_event_message;
static struct ringbuf rxbuf;
static uint8_t rxbuf_data[BUFSIZE];// if is not enough bufsize must be power of 2 (by Guoxuenan)

//=================================

//================================
PROCESS(serial_line_process2, "Serial driver2");
static struct ringbuf rxbuf2;
static uint8_t rxbuf_data2[BUFSIZE];
process_event_t serial_line_event_message2;

/*-=======================by Guoxuenan================================================-*/

int
serial_line_input_byte2(unsigned char c)
{  
   static uint8_t overflow = 0; /* Buffer overflow: ignore until END */


   if(!overflow) 
   {
      /* Add character */
        if(ringbuf_put(&rxbuf2, c) == 0) // 
        {/* Buffer overflow: ignore the rest of the line */
          overflow = 1;
        }
   } 
   else
   {
          /* Buffer overflowed:
           * Only (try to) add terminator characters, otherwise skip */
          if(ringbuf_put(&rxbuf2, c) != 0) 
          {
            overflow = 0;
          }
   }
 //      t0=RTIMER_NOW();

 // while(RTIMER_CLOCK_LT(RTIMER_NOW() , t0 + RTIMER_ARCH_SECOND/20)) {};
   
    process_poll(&serial_line_process2);
     
    //printf("serial line input byte2\n");
   return 1;
}
/*===========================by Guoxuenan========================================================*/

PROCESS_THREAD(serial_line_process2, ev, data)
{
//
  static char buf[BUFSIZE];
  static int ptr;
           
  PROCESS_BEGIN();
 

 serial_line_event_message2 = process_alloc_event();
 ptr = 1;

while(1) 
{
    /* Fill application buffer until newline or empty */

    int c = ringbuf_get(&rxbuf2);
  
    if(c == -1) // 
    {
        /* Buffer empty, wait for poll */
      if(ringbuf_elements(&rxbuf2)==0 && ptr==1)
      {
         PROCESS_YIELD();
      } 
      static rtimer_clock_t t0;
      t0=RTIMER_NOW();
      // 2400while(RTIMER_CLOCK_LT(RTIMER_NOW() , t0 + RTIMER_ARCH_SECOND/20)) {};

      while(RTIMER_CLOCK_LT(RTIMER_NOW() , t0 + RTIMER_ARCH_SECOND/50)) {};
   
      
        if(ringbuf_elements(&rxbuf2)==0)
        { 
              if(ptr!=1)
              {
                  /* Terminate */
                  /* Broadcast event */
                  process_post(PROCESS_BROADCAST, serial_line_event_message2, buf);

                  /* Wait until all processes have handled the serial line event */
                  if(PROCESS_ERR_OK == process_post(PROCESS_CURRENT(), PROCESS_EVENT_CONTINUE, NULL)) 
                  {
                      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
                  }
                  ptr = 1;
                  buf[0]=0;
              }
        }

    }
        
   else 
   {
           if(ptr < BUFSIZE-1) 
           {
                buf[ptr++] = (uint8_t)c;
                buf[0]++;
           } 
           else 
           {

             buf[ptr++] = (uint8_t)c;            
             buf[0]++;
              /* Broadcast event */
              process_post(PROCESS_BROADCAST, serial_line_event_message2, buf);
     
              /* Wait until all processes have handled the serial line event */
              if(PROCESS_ERR_OK == process_post(PROCESS_CURRENT(), PROCESS_EVENT_CONTINUE, NULL)) 
              {
                PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
              }
              ptr = 1;
              buf[0]=0;
            }
    }

}



  PROCESS_END();
}
// 
void
serial_line_init2(void)
{
  ringbuf_init(&rxbuf2, rxbuf_data2, sizeof(rxbuf_data2));
  process_start(&serial_line_process2, NULL);
  //printf("i am executeed----------2-");
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

void
serial_line_init(void)
{
  ringbuf_init(&rxbuf, rxbuf_data, sizeof(rxbuf_data));
  process_start(&serial_line_process, NULL);
  //printf("i am executeed-----------");
}

/*---------------------------------------------------------------------------*/
int
serial_line_input_byte(unsigned char c)
{
  static uint8_t overflow = 0; /* Buffer overflow: ignore until END */
   
  if(IGNORE_CHAR(c)) {
    return 0;
  }

  if(!overflow) {
    /* Add character */
    if(ringbuf_put(&rxbuf, c) == 0) {
      /* Buffer overflow: ignore the rest of the line */
      overflow = 1;
    }
  } else {
    /* Buffer overflowed:
     * Only (try to) add terminator characters, otherwise skip */
    if(c == END && ringbuf_put(&rxbuf, c) != 0) {
      overflow = 0;
    }
  }
  //printf("serial_line_input_byte_char=%c\n",c);
  /* Wake up consumer process */
  process_poll(&serial_line_process);
  return 1;
}
/*---------------------------------------------------------------------------*/


PROCESS_THREAD(serial_line_process, ev, data)
{
  static char buf[BUFSIZE];
  static int ptr;
  //static struct  rtimer rtim;
  PROCESS_BEGIN();

  serial_line_event_message = process_alloc_event();
  ptr = 0;

while(1) 
{
    /* Fill application buffer until newline or empty */
    int c = ringbuf_get(&rxbuf);
   //  printf("process 0 char is %c\n", c);
   // rtimer_set();
    if(c == -1) 
    {
        /* Buffer empty, wait for poll */
        PROCESS_YIELD();
    }
   else 
   {
            if(c != END) 
            {
                if(ptr < BUFSIZE-1) 
                {
                  buf[ptr++] = (uint8_t)c;
                } 
                else 
                {/* Ignore character (wait for EOL) */ }

            } 
           else 
           {
              /* Terminate */
              buf[ptr++] = (uint8_t)'\0';
       
              /* Broadcast event */
              process_post(PROCESS_BROADCAST, serial_line_event_message, buf);

              /* Wait until all processes have handled the serial line event */
              if(PROCESS_ERR_OK == process_post(PROCESS_CURRENT(), PROCESS_EVENT_CONTINUE, NULL)) 
              {
                PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
              }
              ptr = 0;

              

            }
    }

}
 
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
