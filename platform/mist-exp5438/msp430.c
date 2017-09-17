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
 * @(#)$Id: msp430.c,v 1.1 2010/08/24 16:26:38 joxe Exp $
 */
#include "contiki.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "net/ip/uip.h"
#include "hal-pmm.h"

static unsigned long dco_speed;

static void lowpower_init_ports(void);
/*---------------------------------------------------------------------------*/
#if defined(__MSP430__) && defined(__GNUC__) && MSP430_MEMCPY_WORKAROUND
void *
w_memcpy(void *out, const void *in, size_t n)
{
  uint8_t *src, *dest;
  src = (uint8_t *) in;
  dest = (uint8_t *) out;
  while(n-- > 0) {
    *dest++ = *src++;
  }
  return out;
}
#endif /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
/*---------------------------------------------------------------------------*/
#if defined(__MSP430__) && defined(__GNUC__) && MSP430_MEMCPY_WORKAROUND
void *
w_memset(void *out, int value, size_t n)
{
  uint8_t *dest;
  dest = (uint8_t *) out;
  while(n-- > 0) {
    *dest++ = value & 0xff;
  }
  return out;
}
#endif /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
/*---------------------------------------------------------------------------*/
void
msp430_init_dco(void)
{
}
/*---------------------------------------------------------------------------*/
unsigned long
msp430_dco_speed(void)
{
  return dco_speed;
}
/*---------------------------------------------------------------------------*/
/* on fatal error, blink LEDs forever */
static void
panic(void)
{
 // P3DIR |= 0x40;
  while(1) 
  {
  //  P3OUT |= 0x40;
    __delay_cycles(10000);
    __delay_cycles(10000);
    __delay_cycles(10000);
    __delay_cycles(10000);
    __delay_cycles(10000);
   // P3OUT &= ~0x40;
    __delay_cycles(10000);
    __delay_cycles(10000);
    __delay_cycles(10000);
    __delay_cycles(10000);
    __delay_cycles(10000);
  }
}
/*---------------------------------------------------------------------------*/
void
msp430_set_dco_speed(unsigned long mhz)
{
  int multiplier;

  dco_speed = mhz;

  dint();
  /* DCO multiplier m for x MHz:
     (m + 1) * FLLRef = Fdco
     (m + 1) * 32768 = x MHz
     m = x / 32768 - 1
     Set FLL Div = fDCOCLK/2
  */

   // UCSCTL6 |=XCAP_3  ;          //internal load cap

  multiplier = mhz / 32768UL - 1;
  __bis_SR_register(SCG0);

  /* change the core voltage to enable higher clock speed */
  if(SetVCore(PMMCOREV_3) == PMM_STATUS_ERROR) 
  {
    /* unable to set voltage, operation will be unpredictable */
    panic();
    //leds_off(LEDS_GREEN);

  }
  
  //  UCSCTL6 &=~XCAP_3  ;          //internal load cap
    UCSCTL6 |=XCAP_3  ;          //internal load cap

  #if XT2_115200
     UCSCTL6 &= ~(XT2OFF);    //set xt2 on
  #endif
     UCSCTL6 &= ~(XT1OFF);   // set xt1 on
 
  /* changed automatically by the FLL later on */
  UCSCTL0 = 0x0000;
  /* Select DCO range for (at worst) 6--23.7 MHz, (at best) 2.5--54 MHz */
  UCSCTL1 = DCORSEL_5;

  /* Set computed DCO multiplier */
  UCSCTL2 = FLLD_1 + multiplier;
  UCSCTL3 |= SELREF__XT1CLK;        //select xt1clk as FLL reference source. 
  // UCSCTL3 |= SELREF__REFOCLK;//
  
  #if USE_4M_CRYSTAL
    UCSCTL6 &= ~XT2DRIVE0;
  #endif 

  __bic_SR_register(SCG0);

/********clear o fault flag**********************************/
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);// Clear XT2,XT1,DCO fault flags                                       
    SFRIFG1 &= ~OFIFG; 
    //panic();
     __delay_cycles(10000);                        // Clear fault flags
  }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
/********clear fault flag************************************/

  /* Sources for the hardware clocks */
  // UCSCTL4 |= SELA__XT1CLK | SELS__DCOCLKDIV | SELM__DCOCLK;
  UCSCTL6  &=~XT1DRIVE_3;

 #if XT2_115200
  UCSCTL4 |= SELA__XT1CLK | SELS__XT2CLK | SELM__DCOCLK;
 #else
  UCSCTL4 |= SELA__XT1CLK | SELS__DCOCLKDIV | SELM__DCOCLK;
 #endif

  eint();
}
/*---------------------------------------------------------------------------*/
void
msp430_quick_synch_dco(void)
{
  msp430_set_dco_speed(F_CPU);
}
/*---------------------------------------------------------------------------*/
static void 
lowpower_init_ports(void)
{
  //P1.7~P1.4:    gpio 2 3 and  p1.6 1.7 no use , Jtag 12 14 set output low
  P1DIR |= 0xf6;                /*  Direction */
  P1OUT = 0;                /* Output */
  P1REN |= 0xf1;  //P1REN |= 0x31;                /*  Resistor Enable */
  P1DS  = 0x00;
  P1SEL = 0;               /*Selection */
  P1IES = 0;               /* Interrupt Edge Select */
  P1IE = 0;                  /* Interrupt Enable */
  P1IFG = 0;                /* Interrupt Flag */

// p3.7~3.4 : uart 2 pin ; led one pin ;P3.7 for voltage...sample this is necessary

  P3DIR |= 0xff; //P3DIR |= 0xf0;                /*  Direction */
  
  #if HW_NEW_BIG 
    P3OUT |= 0x40; /*P3OUT |= 0x80; */               /* Output */ //set p3.7 output high this is necessary
  #else
  	P3OUT |= 0x80; /*P3OUT |= 0x80; */               /* Output */ //set p3.7 output high this is necessary
  #endif
  
  P3REN = 0xff;                /*  Resistor Enable */
  P3DS  = 0x00;
  P3SEL = 0;               /*Selection */

  P2DIR = 0xff;                /*  Direction */
  P2OUT = 0;                /* Output */
  P2REN = 0xff;                /*  Resistor Enable */
  P2DS  = 0x00;
  P2SEL = 0;               /*Selection */
  P2IES = 0;               /* Interrupt Edge Select */
  P2IE =  0;                  /* Interrupt Enable */
  P2IFG = 0;                /* Interrupt Flag */

  P4DIR = 0xff;                /*  Direction */
  P4OUT = 0;                /* Output */
  P4REN = 0xff;                /*  Resistor Enable */
  P4DS  = 0x00;
  P4SEL = 0;               /*Selection */

  // P5DIR = 0x3f;                /*  Direction */ use uart1
  P5DIR = 0xff;                /*  Direction */
  P5OUT = 0;                /* Output */
  P5REN = 0xff;                /*  Resistor Enable */
  P5DS  = 0x00;
  //P5SEL = 0;               /*Selection */

  //P6IN = 0;                 /*  Input */  
  P6DIR = 0xff;                /*  Direction */
  #if HW_NEW_BIG 
  	P6OUT =0x30 ;  /* P6OUT |=0x10 ; */             //set P6.4 ,output high  this is necessary
  #else
  	P6OUT =0x10 ;  /* P6OUT |=0x10 ; */             //set P6.4 ,output high  this is necessary
  #endif
  P6REN = 0xff;                /*  Resistor Enable */
  P6DS  = 0x00;
  P6SEL = 0;               /*Selection */
  
  //P7IN = 0;                 /*  Input */  
  P7DIR = 0xff;                /*  Direction */
  P7OUT = 0;                /* Output */
  P7REN = 0xff;                /*  Resistor Enable */
  P7DS  = 0x00;
  //P7SEL = 0;               /*Selection */

  //P8IN = 0;                 /*  Input */  
  P8DIR = 0xff;                /*  Direction */
  P8OUT = 0;                /* Output */
  P8REN = 0xff;                /*  Resistor Enable */
  P8DS  = 0X00;
  P8SEL = 0;               /*Selection */


  PJDIR = 0xff;                /*  Direction */
  PJOUT = 0;                 /*Output*/ 
  PJREN = 0xff;                /*  Resistor Enable */
  PJDS  = 0X00;
}
/**************************************************************/
static void
init_ports(void)
{
  /* Turn everything off, device drivers enable what is needed. */

  /* All configured for digital I/O */
#ifdef P1SEL
  P1SEL = 0;
#endif
#ifdef P2SEL
  P2SEL = 0;
#endif
#ifdef P3SEL
  P3SEL = 0;
#endif
#ifdef P4SEL
  P4SEL = 0;
#endif
#ifdef P5SEL
  P5SEL = 0;
  #if XT2_115200
    P5SEL |=0x0c;    //select as xt2 function  p5.2IN  p5.3out
  #endif
#endif
#ifdef P6SEL
  P6SEL = 0;
#endif

  /* All available inputs */
#ifdef P1DIR
  P1DIR = 0;
  P1OUT = 0;
  P1IES = 0;
#endif
#ifdef P2DIR
  P2DIR = 0; /* output needed for the below config ? */
  P2OUT = 0;
  P2IES = 0;
#endif
#ifdef P3DIR
  P3DIR = 0;
  P3OUT = 0;
#endif
#ifdef P4DIR
  P4DIR = 0;
  P4OUT = 0;
#endif

#ifdef P5DIR
  P5DIR = 0;
  P5OUT = 0;
#endif

#ifdef P6DIR
  P6DIR = 0;
  P6OUT = 0;
#endif

#ifdef P7DIR
  P7DIR = 0;
  P7OUT = 0;
  P7SEL |= 0x03;     /* Configure for ext clock function on these pins */
#endif

#ifdef P8DIR
  P8DIR = 0;
  P8OUT = 0;
#endif

  lowpower_init_ports();

  P2SEL|=0x40;      //
  P2DIR|=0x40;
  
  // P1SEL|=0x40;    //8 M 
  // P1DIR|=0x40;

  P1IE = 0;
  P2IE = 0;
}


/*---------------------------------------------------------------------------*/
/* msp430-ld may align _end incorrectly. Workaround in cpu_init. */
#ifndef __IAR_SYSTEMS_ICC__
extern int _end;                /* Not in sys/unistd.h */
static char *cur_break = (char *)&_end;
#endif

void
msp430_cpu_init(void)
{
  dint();
  watchdog_init();
  init_ports();
  dco_speed = 1048576; /* Default bootup DCO frequency */
  msp430_quick_synch_dco();
  eint();
#ifndef __IAR_SYSTEMS_ICC__
  if((uintptr_t)cur_break & 1) { /* Workaround for msp430-ld bug! */
    cur_break++;
  }
#endif
}
/*---------------------------------------------------------------------------*/
#define asmv(arg) __asm__ __volatile__(arg)

#define STACK_EXTRA 32

/*
 * Allocate memory from the heap. Check that we don't collide with the
 * stack right now (some other routine might later). A watchdog might
 * be used to check if cur_break and the stack pointer meet during
 * runtime.
 */

#if 0
void *
sbrk(int incr)
{
  char *stack_pointer;
#ifdef __IAR_SYSTEMS_ICC__
  stack_pointer = (char *) __get_SP_register();
  /* TODO: add code here... */
  return 0;
#else
  asmv("mov r1, %0" : "=r" (stack_pointer));
  stack_pointer -= STACK_EXTRA;
  if(incr > (stack_pointer - cur_break))
    return (void *)-1;          /* ENOMEM */

  void *old_break = cur_break;
  cur_break += incr;
  /*
   * If the stack was never here then [old_break .. cur_break] should
   * be filled with zeros.
  */
  return old_break;
#endif
}
#endif
/*---------------------------------------------------------------------------*/
/*
 * Mask all interrupts that can be masked.
 */
int
splhigh_(void)
{
  /* Clear the GIE (General Interrupt Enable) flag. */
  int sr;
#ifdef __IAR_SYSTEMS_ICC__
  sr = __get_SR_register();
  __bic_SR_register(GIE);
#else
  asmv("mov r2, %0" : "=r" (sr));
  asmv("bic %0, r2" : : "i" (GIE));
#endif
  return sr & GIE;              /* Ignore other sr bits. */
}
/*---------------------------------------------------------------------------*/
/*
 * Restore previous interrupt mask.
 */
void
splx_(int sr)
{
  /* If GIE was set, restore it. */
#ifdef __IAR_SYSTEMS_ICC__
  __bis_SR_register(sr);
#else
  asmv("bis %0, r2" : : "r" (sr));
#endif
}

/*---------------------------------------------------------------------------*/
#ifdef __IAR_SYSTEMS_ICC__
uint16_t last_sysrstiv;
int __low_level_init(void)
{
  /* turn off watchdog so that C-init will run */
  WDTCTL = WDTPW + WDTHOLD;
  
  /* save reset IV register, will contain cause of last reset (for debugging) */
  last_sysrstiv = SYSRSTIV;
  /*
   * Return value:
   *
   *  1 - Perform data segment initialization.
   *  0 - Skip data segment initialization.
   */

  return 1;
}
#endif
/*---------------------------------------------------------------------------*/
void
msp430_sync_dco(void)
{
}
/*---------------------------------------------------------------------------*/