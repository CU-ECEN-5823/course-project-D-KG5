/*
 * adc.c
 *
 *  Created on: Nov 11, 2019
 *      Author: Dhruva
 */

#include <stdio.h>
#include "adc.h"
#include "ldma.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_gpio.h"
#include "em_letimer.h"

/**
 * Inspired by SiliconLabs/peripheral_examples by silabs-DavidS et al.
 * https://github.com/SiliconLabs/peripheral_examples
 */
void adc_init(void){
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef initSingle = {
		adcPRSSELCh0,            /* PRS ch0 (if enabled). */                          \
		adcAcqTime4,             /* 1 ADC_CLK cycle acquisition time. */              \
		adcRef2V5,               /* 1.25 V internal reference. */                     \
		adcRes12Bit,             /* 12 bit resolution. */                             \
		adcPosSelAPORT2XCH9,    /* Select node BUS2XCH9 as posSel */                \
		adcNegSelVSS,            /* Select VSS as negSel */                           \
		false,                   /* Single-ended input. */                            \
		true,                   /* PRS enabled. */                                  \
		false,                   /* Right adjust. */                                  \
		false,                   /* Deactivate conversion after one scan sequence. */ \
		false,                   /* No EM2 DMA wakeup from single FIFO DVL */         \
		false                    /* Discard new data on full FIFO. */                 \
	};

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_ADC0, true);

	CMU->ADCCTRL = CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO; /* select AUXHFRCO for ADC ASYNC for EM2 */

	/* set up ADC with AUXHFRCO frequency */
	CMU_AUXHFRCOFreqSet(cmuAUXHFRCOFreq_4M0Hz);
	init.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
	init.prescale = ADC_PrescaleCalc(ADC_FREQ, CMU_AUXHFRCOBandGet());

	init.em2ClockConfig = adcEm2ClockOnDemand;
	initSingle.singleDmaEm2Wu = 1;

	initSingle.prsSel = (ADC_PRSSEL_TypeDef) PRS_CHANNEL;	/* select channel 0 */

	ADC_Init(ADC0, &init);
	ADC_InitSingle(ADC0, &initSingle);

	/* set single data valid level ad clear single FIFO */
	ADC0->SINGLECTRLX = (ADC0->SINGLECTRLX & ~_ADC_SINGLECTRLX_DVL_MASK) | (((ADC_DVL - 1) << _ADC_SINGLECTRLX_DVL_SHIFT) & _ADC_SINGLECTRLX_DVL_MASK);
	ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;
}
