/*
 * adc.h
 *
 *  Created on: Nov 11, 2019
 *      Author: Dhruva
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_FREQ	16000000
#define ADC_BUFFER_SIZE 4
#define ADC_DVL 2

uint32_t adcBuffer[ADC_BUFFER_SIZE];

void adc_init(void);

#endif /* ADC_H_ */
