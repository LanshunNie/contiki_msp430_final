/*
 * Copyright (c) 2006, Swedish Institute of Computer Science
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
 * @(#)$Id: spix.c,v 1.1 2010/08/24 16:23:20 joxe Exp $
 */

#include "contiki-conf.h"

/*
 * This is SPI initialization code for the MSP430X architecture.
 *
 */
/* unsigned char spi_busy = 0; */
/*---------------------------------------------------------------------------*/
/*
 * Initialize SPI bus.
 */
void
spi_init(void)
{
  /* Initalize ports for communication with SPI units. */

  UCB0CTL1 |= UCSWRST;                /* Reset USCI */
  UCB0CTL1 |= UCSSEL_2;               /* smclk while usci is reset */
  /* MSB-first 8-bit, Master, Synchronous, 3 pin SPI master, no ste,
     watch-out for clock-phase UCCKPH */
  //  UCB0CTL0 = (UCMSB | UCMST | UCSYNC | UCCKPL);

  UCB0CTL0 |= UCMST | UCMSB | UCSYNC | UCCKPL | UCMODE_1;

  /*
   * Set up SPI bus speed. If CPU is too fast, we need to lower speed to
   * accomodate CC11xx max SPI speeds.
   */
  UCB0BR1 = 0x00;
#if F_CPU==16000000uL
  /* Baud rate is F_CPU/4 */
  UCB0BR0 = 0x02;
#elif F_CPU==8000000uL
  /* Baud rate is F_CPU/2 */
  UCB0BR0 = 0x01;
#else
#error Unknown SPI UBR config for F_CPU, please configure and verify SPI comm.
#endif

  /* Dont need modulation control. */
  /* UCB0MCTL = 0; */

  /* Select Peripheral functionality */
  // SCK = 3   P3.3 - Output: SPI Serial Clock (SCLK)
  // MISO = 2  P3.2 - Input:  SPI Master in - slave out (MISO)
  // MOSI = 1  P3.1 - Output: SPI Master out - slave in (MOSI)

  P3SEL |= BV(SCK) | BV(MOSI) | BV(MISO);
  /* Configure as outputs(SIMO,CLK). */
  P3DIR |= BV(SCK) | BV(MISO);

  /* Clear pending interrupts before enabling. */
  UCB0IE &= ~UCRXIFG;
  UCB0IE &= ~UCTXIFG;
  /* Remove RESET before enabling interrupts */
  UCB0CTL1 &= ~UCSWRST;

  /* Enable UCB0 Interrupts */
  /* Enable USCI_B0 TX Interrupts */
  /* IE2 |= UCB0TXIE; */
  /* Enable USCI_B0 RX Interrupts */
  /* IE2 |= UCB0RXIE; */

  // keep reset high
  P1SEL &= ~0x08; //P1.3 as I/O function
  P1DIR |= 0x08;  //P1.3 as output
  P1OUT |= 0x08;  //P1.3 high  
}
/*---------------------------------------------------------------------------*/