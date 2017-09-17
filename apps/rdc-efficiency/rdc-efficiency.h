/**
 * \addtogroup orpl energy efficiency process
 * @{
 */

/*
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: rdc-efficiency.h,v 1.21 2016/12/19 16:47:38 
 */

/**
 * \file
 *         A simple rdc efficiency mechanism
 * \author
 *         zhangwei
 */


#ifndef __RDC_EFFICIENCY_H__
#define __RDC_EFFICIENCY_H__

#include "net/mac/energy-efficiency/energy-efficiency.h"

void rdc_efficiency_request_poll(void);

void rdc_efficiency_init(void);

extern  process_event_t energy_efficient_begin_event;

#endif /* __RDC_EFFICIENCY_H__ */

