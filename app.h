/***************************************************************************//**
 * @file  app.h
 * @brief Application header file
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include <gecko_configuration.h>
#include "native_gecko.h"

/***************************************************************************//**
 * @defgroup app Application Code
 * @brief Sample Application Implementation
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Application
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup app
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * Main application code.
 * @param[in] pConfig  Pointer to stack configuration.
 ******************************************************************************/
void appMain(gecko_configuration_t *pConfig);
uint16_t map(uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max, uint32_t s);
uint32_t adcAvg;
uint16_t get_adc();
void lpn_init(void);

/// lpn state
static PACKSTRUCT(struct lpn_state {
  // On/Off Server state
  uint8_t onoff_current;          /**< Current generic on/off value */
  uint8_t onoff_target;           /**< Target generic on/off value */

  // ADC server
  uint16_t adc_current;   /**< Current adc value */
  uint16_t adc_previous;    /**< Target adc value */
  uint16_t adc_default;   /**< Default adc value */
  uint16_t adc_min;       /**< Minimum adc value */
  uint16_t adc_max;       /**< Maximum adc value */
}) lpn_state;

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */

#endif /* APP_H */
