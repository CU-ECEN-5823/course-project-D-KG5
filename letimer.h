/*
 * letimer.h
 *
 *  Created on: Sep 8, 2019
 *      Author: Dhruva
 */

#ifndef SRC_LETIMER_H_
#define SRC_LETIMER_H_

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_gpio.h"
#include "em_letimer.h"
#include "em_prs.h"
#include "em_ldma.h"

void letimer_init(void);

/* set buffer fill frequency to 1 averaged value per second */
#ifndef INT_FREQ
#define INT_FREQ 4
#endif

/* convert from ms to ticks */
#define TICKS_IN_MS(X)        32768 * (X) / 1000

#endif /* SRC_LETIMER_H_ */
