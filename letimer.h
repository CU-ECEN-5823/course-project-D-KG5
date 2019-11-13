/*
 * letimer.h
 *
 *  Created on: Sep 8, 2019
 *      Author: Dhruva
 */

#ifndef SRC_LETIMER_H_
#define SRC_LETIMER_H_

void letimer_init(void);

/* set blink period in ms */
#ifndef PERIOD
#define PERIOD 1000
#endif

/* convert from ms to ticks */
#define TICKS_IN_MS(X)        32768 * (X) / 1000

///** Application timer enumeration. */
//typedef enum {
//  /* Timer for toggling the the EXTCOMIN signal for the LCD display */
//  DISP_UPDATE_TIMER,
//  FACTORY_RESET_TIMER,
//  ID_RETRANS_TIMER
//} swTimer_t;

#endif /* SRC_LETIMER_H_ */
