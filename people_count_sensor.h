/***************************************************************************//**
 * @file  people_count_sensor.h
 * @brief People count sensor header
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

#ifndef PEOPLE_COUNT_SENSOR_H
#define PEOPLE_COUNT_SENSOR_H

#include "mesh_device_properties.h"

/***************************************************************************//**
 * @defgroup PeopleCount People Count Sensor Module
 * @brief People Count Sensor Module Implementation
 * This module simulate the people count sensor behavior.
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Sensor
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup PeopleCount
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * Set the people count value. It could be used to initialize the sensor.
 *
 * @param[in] people_count  People count value to set
 ******************************************************************************/
void set_people_count(count16_t people_count);

/***************************************************************************//**
 * Get the current people count value measured by sensor.
 *
 * @return Current value of people count.
 ******************************************************************************/
count16_t get_people_count(void);

/***************************************************************************//**
 * Increase people count value by one. After exceeding the maximum value it set
 * people count to value is not known.
 ******************************************************************************/
void people_count_increase(void);

/***************************************************************************//**
 * Decrease people count value by one if value is known and greater than 0.
 ******************************************************************************/
void people_count_decrease(void);

/** @} (end addtogroup PeopleCount) */
/** @} (end addtogroup Sensor) */

#endif /* PEOPLE_COUNT_SENSOR_H */
