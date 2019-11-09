/***************************************************************************//**
 * @file  temperature_sensor.h
 * @brief Temperature sensor header
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

#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "mesh_device_properties.h"

/***************************************************************************//**
 * @defgroup TemperatureSensor Temperature Sensor Module
 * @brief Temperature Sensor Module Implementation
 * This module implements temperature sensor behavior.
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Sensor
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup TemperatureSensor
 * @{
 ******************************************************************************/

/*******************************************************************************
 * Initialize the temperature sensor.
 ******************************************************************************/
void init_temperature_sensor();

/*******************************************************************************
 * Get the current temperature value measured by sensor.
 *
 * @return Current value of temperature.
 ******************************************************************************/
temperature_8_t get_temperature(void);

/** @} (end addtogroup TemperatureSensor) */
/** @} (end addtogroup Sensor) */

#endif /* TEMPERATURE_SENSOR_H */
