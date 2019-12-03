/*
 * ldma.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Dhruva
 */
#include "ldma.h"
#include "adc.h"

void LDMA_IRQHandler(void){
	LDMA_IntClear((1 << LDMA_CHANNEL) << _LDMA_IFC_DONE_SHIFT);	/* clear interrupts */
	gecko_external_signal(EXT_SIGNAL_LDMA_INT);
}

/**
 * Inspired by SiliconLabs/peripheral_examples by silabs-DavidS et al.
 * https://github.com/SiliconLabs/peripheral_examples
 */
void ldma_init(void){
	CMU_ClockEnable(cmuClock_LDMA, true);

	LDMA_Init_t LDMA_INITIAL_CONFIG = LDMA_INIT_DEFAULT;
	LDMA_Init(&LDMA_INITIAL_CONFIG);

	trans = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_ADC0_SINGLE);	/* transfer trigger off ADC single conversion complete */

	descr = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_BYTE(
			&(ADC0->SINGLEDATA),	/* src */
			adcBuffer,				/* dst */
			ADC_BUFFER_SIZE,		/* data trasnfer size */
			0);						/* link to self */

	descr.xfer.blockSize = ADC_DVL - 1; /* transfers ADC_DVL number of units per arb cycle */
	descr.xfer.ignoreSrec = true;	/* ignore single requests */
	descr.xfer.size = ldmaCtrlSizeWord;	/* transfer words */

	LDMA_StartTransfer(LDMA_CHANNEL, &trans, &descr);

	NVIC_ClearPendingIRQ(LDMA_IRQn);
	NVIC_EnableIRQ(LDMA_IRQn);
}
