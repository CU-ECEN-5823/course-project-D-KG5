/*
 * letimer.c
 *
 *  Created on: Sep 8, 2019
 *      Author: Dhruva
 */
#include <stdio.h>
#include "letimer.h"
#include "ldma.h"

/**
 * Inspired by SiliconLabs/peripheral_examples by silabs-DavidS et al.
 * https://github.com/SiliconLabs/peripheral_examples
 */
void letimer_init(void){
	/* select LFRCO */
	CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
	CMU_ClockEnable(cmuClock_HFLE, true);	/* enable HFLE for low energy */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);	/* set up LFRCO clock source and LETIMER0 clock */
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	/* set LETIMER config struct */
	const LETIMER_Init_TypeDef LETIMER_INITIAL_CONFIG = {
		.enable = true,
		.debugRun = true,
		.comp0Top = true,	/* load COMP0 into CNT when underflowing */
		.bufTop = false,	/* dont need because repeat is free */
		.ufoa0 = letimerUFOAPulse,
		.repMode = letimerRepeatFree
	};

	LETIMER_Init(LETIMER0, &LETIMER_INITIAL_CONFIG);

	LETIMER_RepeatSet(LETIMER0, 0, 1);	/* pulse on underflow */
	LETIMER_CompareSet(LETIMER0, 0, CMU_ClockFreqGet(cmuClock_LETIMER0) / INT_FREQ);	/* compare on wake-up count */

	CMU_ClockEnable(cmuClock_PRS, true);	/* enable ASYBC PRS using LETIMER0 to trigger ADC in EM2 */
	PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_CH_CTRL_SOURCESEL_LETIMER0, PRS_CH_CTRL_SIGSEL_LETIMER0CH0);
}

