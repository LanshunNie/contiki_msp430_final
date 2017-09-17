/*
 * Copyright (c) 2010, Swedish Institute of Computer Science
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
 * @(#)$Id: uart1x.c,v 1.1 2010/08/24 16:23:20 joxe Exp $
 */

/*
 * Yet another machine dependent MSP430X UART1 code.
 * IF2, etc. can not be used here... need to abstract to some macros
 * later.
 */

#include "contiki.h"
#include <stdlib.h>

#include "sys/energest.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "lib/ringbuf.h"
#include "dev/leds.h"
#include "isr_compat.h"

static int (*uart1_input_handler)(unsigned char c);

static volatile uint8_t transmitting;


/*---------------------------------------------------------------------------*/
uint8_t
uart1_active(void)
{
  return (UCA1STAT & UCBUSY) | transmitting;
}
/*---------------------------------------------------------------------------*/
void
uart1_set_input(int (*input)(unsigned char c))
{
  uart1_input_handler = input;
}
/*---------------------------------------------------------------------------*/
void
uart1_writeb(unsigned char c)
{
  watchdog_periodic();
  /* Loop until the transmission buffer is available. */
  while((UCA1STAT & UCBUSY));

  /* Transmit the data. */
  UCA1TXBUF = c;
}
/*---------------------------------------------------------------------------*/
#if ! WITH_UIP /* If WITH_UIP is defined, putchar() is defined by the SLIP driver */
#endif /* ! WITH_UIP */
/*---------------------------------------------------------------------------*/
/**
 * Initalize the RS232 port.
 *
 */
void
uart1_init(unsigned long ubr)
{
  #if XT2_115200
#if USE_4M_CRYSTAL
    ubr = 4000000UL / 115200UL;
    #else
    ubr = 8000000UL / 115200UL;
    #endif 
  #elif HEAT_METER
    ubr = 32768UL / 2400UL;
  #else 
    ubr = 32768UL / 9600UL;
  #endif

  /* RS232 */
  UCA1CTL1 |= UCSWRST;            /* Hold peripheral in reset state */
  #if XT2_115200
     UCA1CTL1 |= UCSSEL_2;           /* CLK = SMCLK */
  #elif HEAT_METER 
      UCA1CTL1 |= UCSSEL_1;
  #else
      UCA1CTL1 |= UCSSEL_1;
  #endif
  /* UCA1BR0 = 0x45;                 /\* 8MHz/115200 = 69 = 0x45 *\/ */
  UCA1BR0 = ubr & 0xff; //0x45; /* tested... */
  UCA1BR1 = ubr >> 8;
 
  #if HEAT_METER
  UCA1CTL0 |=UCPEN+UCPAR;
  #endif 

  #if XT2_115200
  #if USE_4M_CRYSTAL
    UCA1MCTL |= UCBRS_6 + UCBRF_0;
   #else
    UCA1MCTL |= UCBRS_4 + UCBRF_0;  //orgin 7 3
   #endif
  #elif HEAT_METER
   UCA1MCTL |= UCBRS_6 + UCBRF_0;  //orgin 3 0 
  #else
    UCA1MCTL |= UCBRS_3 + UCBRF_0;
  #endif

  //UCA1MCTL = UCBRS_3;             /* Modulation UCBRSx = 3 */
  P5DIR &= ~0x80;                 /* P5.7 = USCI_A1 RXD as input */
  P5DIR |= 0x40;                  /* P5.6 = USCI_A1 TXD as output */
  P5SEL |= 0xc0;                  /* P5.6,7 = USCI_A1 TXD/RXD */

  /*UCA1CTL1 &= ~UCSWRST;*/       /* Initialize USCI state machine */

  transmitting = 0;

  /* XXX Clear pending interrupts before enable */
  UCA1IE &= ~UCRXIFG;
  UCA1IE &= ~UCTXIFG;

  UCA1CTL1 &= ~UCSWRST;                   /* Initialize USCI state machine **before** enabling interrupts */
  UCA1IE |= UCRXIE;                        /* Enable UCA1 RX interrupt */
}

interrupt(USCI_A1_VECTOR)
uart1_rx_interrupt(void){

uint8_t c;

  if(UCA1IV == 2) {

    if(UCA1STAT & UCRXERR) {
      c = UCA1RXBUF;   // Clear error flags by forcing a dummy read. 
    } else {
      c = UCA1RXBUF;
      if(uart1_input_handler != NULL) {
        if(uart1_input_handler(c)) {
          LPM4_EXIT;
        }
      }
    }
  }
 // ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}


/*---------------------------------------------------------------------------*/
/*ISR(USCI_A1, uart1_rx_interrupt)
{
  uint8_t c;

  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  //leds_toggle(LEDS_ALL);
  if(UCA1IV == 2) {
    if(UCA1STAT & UCRXERR) {
      c = UCA1RXBUF;   // Clear error flags by forcing a dummy read. 
    } else {
      c = UCA1RXBUF;
      if(uart1_input_handler != NULL) {
        if(uart1_input_handler(c)) {
          LPM4_EXIT;
        }
      }
    }
  }
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
} */
/*---------------------------------------------------------------------------*/
// /*/*
// void
// uart1_init(unsigned long ubr)
// {
//   #if XT2_115200
//     ubr = 8000000UL / 115200UL;
//   #else 
//     ubr = 32768UL / 9600UL;
//   #endif

//   /* RS232 */
//   UCA1CTL1 |= UCSWRST;            /* Hold peripheral in reset state */
//   #if XT2_115200
//      UCA1CTL1 |= UCSSEL_2;           /* CLK = SMCLK */
//   #else 
//      UCA1CTL1 |= UCSSEL_1;
//   #endif
//   /* UCA1BR0 = 0x45;                 /\* 8MHz/115200 = 69 = 0x45 *\/ */
//   UCA1BR0 = ubr & 0xff; //0x45; /* tested... */
//   /* UCA1BR0 = 9; */
  

//   UCA1BR1 = ubr >> 8;
//   #if XT2_115200
//      UCA1MCTL |= UCBRS_7 + UCBRF_0;
//   #else
//     UCA1MCTL |= UCBRS_3 + UCBRF_0;
//   #endif

//   //UCA1MCTL = UCBRS_3;             /* Modulation UCBRSx = 3 */
//   P5DIR &= ~0x80;                 /* P5.7 = USCI_A1 RXD as input */
//   P5DIR |= 0x40;                  /* P5.6 = USCI_A1 TXD as output */
//   P5SEL |= 0xc0;                  /* P5.6,7 = USCI_A1 TXD/RXD */

//   /*UCA1CTL1 &= ~UCSWRST;*/       /* Initialize USCI state machine */

//   transmitting = 0;

//   /* XXX Clear pending interrupts before enable */
//   UCA1IE &= ~UCRXIFG;
//   UCA1IE &= ~UCTXIFG;

//   UCA1CTL1 &= ~UCSWRST;                   /* Initialize USCI state machine **before** enabling interrupts */
//   UCA1IE |= UCRXIE;                        /* Enable UCA1 RX interrupt */
// }*/*/