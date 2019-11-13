/*
 * letimer.c
 *
 *  Created on: Sep 8, 2019
 *      Author: Dhruva
 */
#include <stdio.h>
#include "letimer.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_gpio.h"
#include "em_letimer.h"

void letimer_init(void){
	/* select ULFRCO and set prescaler */
	CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);	/* set up ULFRCO clock source and LETIMER0 clock */
	CMU_ClockDivSet(cmuClock_LFA, 1);
	CMU_ClockEnable(cmuClock_LFA, true);
	CMU_ClockDivSet(cmuClock_LETIMER0, 1);	/* set prescaler to 1 so no change in clk rate */
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	/* set COMP0 (period) register */
	LETIMER_CompareSet(LETIMER0, 0, (CMU_ClockFreqGet(cmuClock_LETIMER0)) * (PERIOD) / 1000);

	/* set LETIMER config struct */
	const LETIMER_Init_TypeDef LETIMER_INITIAL_CONFIG = {
		.enable = true,
		.debugRun = true,
		.comp0Top = true,	/* load COMP0 into CNT when underflowing */
		.bufTop = false,	/* dont need because repeat is free */
		.ufoa0 = letimerUFOANone,
		.ufoa1 = letimerUFOANone,
		.repMode = letimerRepeatFree
	};

	LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
	LETIMER_Init(LETIMER0, &LETIMER_INITIAL_CONFIG);

	NVIC_ClearPendingIRQ(LETIMER0_IRQn);
	NVIC_EnableIRQ(LETIMER0_IRQn);
}

void LETIMER0_IRQHandler(void){
//	printf("LETIMER handler\r\n");
	uint32_t reason = LETIMER_IntGet(LETIMER0);	/* get interrupt flag and clear it */
	if((reason & LETIMER_IF_UF) == LETIMER_IF_UF){
		LETIMER_IntClear(LETIMER0, reason);
		ADC_Start(ADC0, adcStartSingle);
	}
}

