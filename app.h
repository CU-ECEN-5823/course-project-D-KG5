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
uint16_t adcAvgmapped;
uint32_t adcAvg;
uint16_t get_adc();

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */

#endif /* APP_H */
