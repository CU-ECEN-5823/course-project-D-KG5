/*
 * ldma.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Dhruva
 */

#ifndef LDMA_H_
#define LDMA_H_

#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_prs.h"
#include "em_ldma.h"
#include "em_letimer.h"
#include "native_gecko.h"

#define LDMA_CHANNEL 0
#define PRS_CHANNEL 0

#define EXT_SIGNAL_LDMA_INT      0x04

LDMA_TransferCfg_t trans;
LDMA_Descriptor_t descr;

void ldma_init(void);

#endif /* LDMA_H_ */
