/*
 * adc.c
 *
 *  Created on: Nov 11, 2019
 *      Author: Dhruva
 */

#include <stdio.h>
#include "adc.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_gpio.h"
#include "em_letimer.h"

void adc_init(void){
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_ADC0, true);

	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	const ADC_InitSingle_TypeDef initSingle = {
		adcPRSSELCh0,            /* PRS ch0 (if enabled). */                          \
		adcAcqTime4,             /* 1 ADC_CLK cycle acquisition time. */              \
		adcRef2V5,               /* 1.25 V internal reference. */                     \
		adcRes12Bit,             /* 12 bit resolution. */                             \
		adcPosSelAPORT2XCH9,    /* Select node BUS3XCH10 as posSel */                \
		adcNegSelVSS,            /* Select VSS as negSel */                           \
		false,                   /* Single-ended input. */                            \
		false,                   /* PRS disabled. */                                  \
		false,                   /* Right adjust. */                                  \
		false,                   /* Deactivate conversion after one scan sequence. */ \
		false,                   /* No EM2 DMA wakeup from single FIFO DVL */         \
		false                    /* Discard new data on full FIFO. */                 \
	};

	init.prescale = ADC_PrescaleCalc(ADC_FREQ, 0);
	init.timebase = ADC_TimebaseCalc(0);

	ADC_Init(ADC0, &init);
	ADC_InitSingle(ADC0, &initSingle);

	ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

	NVIC_ClearPendingIRQ(ADC0_IRQn);
	NVIC_EnableIRQ(ADC0_IRQn);
}

void ADC0_IRQHandler(void){
	sample = ADC_DataSingleGet(ADC0);
	mV = ((sample * 2500) / 4096) - 0;
	printf("mV: %lu\r\n", mV);
}
