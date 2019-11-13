/*
 * adc.h
 *
 *  Created on: Nov 11, 2019
 *      Author: Dhruva
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_FREQ	16000000

volatile uint32_t sample;
volatile uint32_t mV;

void adc_init(void);

#endif /* ADC_H_ */
