#ifndef __ADC_H__
#define __ADC_H__
#include <inttypes.h>

/*
* @Author: Guoxuenan
* @Date:   2016-07-31 17:18:26
* @Last Modified by:   Guoxuenan
* @Last Modified time: 2016-07-31 17:22:42
*/


void adc_initial(void);
void adc_start(void);
void adc_close(void);
uint16_t  get_voltage(void);

#endif