
#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#undef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE 200

#undef UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS (UIP_CONF_BUFFER_SIZE - 70)
#undef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW UIP_CONF_TCP_MSS

/* CPU target speed in Hz; works fine at 8, 16, 18 MHz but not higher. */
#define F_CPU 16000000uL

/* Our clock resolution, this is the same as Unix HZ. */
#define CLOCK_CONF_SECOND 128UL

#define BAUD2UBR(baud) ((F_CPU/baud))

#define CCIF
#define CLIF

#define HAVE_STDINT_H
#define MSP430_MEMCPY_WORKAROUND 1
#if defined(__MSP430__) && defined(__GNUC__) && MSP430_MEMCPY_WORKAROUND
#else /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
#define w_memcpy memcpy
#endif /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
#include "msp430def.h"

/* Types for clocks and uip_stats */
typedef unsigned short uip_stats_t;
typedef unsigned long clock_time_t;
typedef unsigned long off_t;

// /* 32-bit rtimer */
// #define RTIMER_CONF_SECOND (4096UL*8)
// typedef uint32_t rtimer_clock_t;
// #define RTIMER_CLOCK_LT(a,b)        ((int32_t)(((rtimer_clock_t)a)-((rtimer_clock_t)b)) < 0) //(((((rtimer_clock_t)a)-((rtimer_clock_t)b))>>31)&1) 

/* DCO speed resynchronization for more robust UART, etc. */
/* Not needed from MSP430x5xx since it make use of the FLL */
#define DCOSYNCH_CONF_ENABLED 0
#define DCOSYNCH_CONF_PERIOD 30

#define ROM_ERASE_UNIT_SIZE  512
#define XMEM_ERASE_UNIT_SIZE (64*1024L)

#define CFS_CONF_OFFSET_TYPE    long

/* Use the first 64k of external flash for node configuration */
#define NODE_ID_XMEM_OFFSET     (0 * XMEM_ERASE_UNIT_SIZE)

/* Use the second 64k of external flash for codeprop. */
#define EEPROMFS_ADDR_CODEPROP  (1 * XMEM_ERASE_UNIT_SIZE)

#define CFS_XMEM_CONF_OFFSET    (2 * XMEM_ERASE_UNIT_SIZE)
#define CFS_XMEM_CONF_SIZE      (1 * XMEM_ERASE_UNIT_SIZE)

#define CFS_RAM_CONF_SIZE 4096

/*
 * SPI bus configuration
 */

/* SPI input/output registers. */
#define SPI_TXBUF UCB0TXBUF
#define SPI_RXBUF UCB0RXBUF

                                /* USART0 Tx ready? */
#define SPI_WAITFOREOTx() while ((UCB0STAT & UCBUSY) != 0)
                                /* USART0 Rx ready? */
#define SPI_WAITFOREORx() while ((UCB0IFG & UCRXIFG) == 0)
                                /* USART0 Tx buffer ready? */
#define SPI_WAITFORTxREADY() while ((UCB0IFG & UCTXIFG) == 0)

#define MOSI           1  /* P3.1 - Output: SPI Master out - slave in (MOSI) */
#define MISO           2  /* P3.2 - Input:  SPI Master in - slave out (MISO) */
#define SCK            3  /* P3.3 - Output: SPI Serial Clock (SCLK) */

#define CC1120_ARCH_SPI_ENABLE  cc1120_arch_spi_enable
#define CC1120_ARCH_SPI_DISABLE cc1120_arch_spi_disable
#define CC1120_ARCH_SPI_RW_BYTE cc1120_arch_spi_rw_byte
#define CC1120_ARCH_SPI_RW      cc1120_arch_spi_rw


#define NULLRDC_CONF_ACK_WAIT_TIME                RTIMER_SECOND / 625
#define NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME RTIMER_SECOND / 1000

#define NETSTACK_RADIO_MAX_PAYLOAD_LEN 125

//signed char cc1120_read_rssi(void);
void cc1120_channel_set(uint8_t c);
#define MULTICHAN_CONF_SET_CHANNEL(x) cc1120_channel_set(x)
//#define MULTICHAN_CONF_READ_RSSI(x) cc1120_read_rssi()

#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM        8

#undef SICSLOWPAN_CONF_MAXAGE
#define SICSLOWPAN_CONF_MAXAGE 16

#define  COOJA_SIMULATION  0

#ifndef  CONTIKI_TARGET_TRXEB1120
#define  CONTIKI_TARGET_TRXEB1120  1
#endif
/*------------------------------------------------*/
/*------------time synch configure----------------*/

#define CONTIKI_CONF_NETSYNCH  1
#if CONTIKI_CONF_NETSYNCH
#define SCHEDULE_SIZE 18
#endif
/*------------------------------------------------*/
/**-------------low power configure ---------------*/
#ifndef TRXEB1120_CONF_LOWPOWER
#define TRXEB1120_CONF_LOWPOWER  1
#endif

#if TRXEB1120_CONF_LOWPOWER
/*no need probe */ 
#undef RPL_CONF_WITH_PROBING
#define RPL_CONF_WITH_PROBING      0   

/* RPL does not use RPL's downwards routing */
#undef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_NO_DOWNWARD_ROUTES

/* RPL does not use routing entries */
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES  0

#define NETSTACK_CONF_RDC_CHANNEL_CHECK_INTERVAL  20

// conf if support change dio interval
#ifndef RPL_CONF_CHANGE_DIO_INTERVAL
#define RPL_CONF_CHANGE_DIO_INTERVAL  0
#endif

#undef  RPL_CONF_DIS_INTERVAL
#define RPL_CONF_DIS_INTERVAL   (1*60)    // 300s

#undef  RPL_CONF_MAX_DAG_PER_INSTANCE
#define RPL_CONF_MAX_DAG_PER_INSTANCE   1

#undef  UIP_CONF_SUPPORT_ECHO
#define UIP_CONF_SUPPORT_ECHO  0

#undef  UIP_CONF_SUPPORT_ICMP6_ERROR_OUTPUT
#define UIP_CONF_SUPPORT_ICMP6_ERROR_OUTPUT	  0

/** Period for uip-ds6 periodic task*/
#ifndef UIP_DS6_CONF_PERIOD
#define UIP_DS6_CONF_PERIOD   (CLOCK_SECOND)
#endif

//wake dev : complie example/ipv6/multicast-new root.c file
#endif /* TRXEB1120_CONF_LOWPOWER */

/*------------------------------------------------*/
/*-------------uart  configure -------------------*/
#undef  ROOTNODE
#define ROOTNODE  0

#ifndef ROOTNODE_CORRECTTIME_TIMEOUT_REBOOT
#define ROOTNODE_CORRECTTIME_TIMEOUT_REBOOT  1
#endif

#if 1
#define WAKEUP_NODE    1   
#define WAKEUP_NODE_RFCHANNEL  2    //4
#endif /*1 */

#ifndef HW_NEW_BIG
#define HW_NEW_BIG  0
#endif 

#ifndef HW_CONF_WITH_UART1
#define HW_CONF_WITH_UART1 0
#endif

#if HW_CONF_WITH_UART1
#define uart_active()  uart1_active()
#else
#define uart_active()  uart0_active()
#endif

// default baud rate 9600
#if ROOTNODE
	#define XT2_115200     1     // baud rate 115200
	#define USE_4M_CRYSTAL 0  //0
	#define ROOT_WITH_ENERGY_EFFICIENCY 1
#else
	#define WITH_ENERGY_EFFICIENCY 1
#endif

#define HEAT_METER 0 // baud rate 2400
/*------------------------------------------------*/
#endif /* __PLATFORM_CONF_H__ */




