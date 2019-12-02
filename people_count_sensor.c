/***************************************************************************//**
 * @file  people_count_sensor.c
 * @brief People count sensor implementation
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

#include <stdio.h>
#include "people_count_sensor.h"
#include "display_interface.h"

/***************************************************************************//**
 * @addtogroup Sensor
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup PeopleCount
 * @{
 ******************************************************************************/

#define VALUE_IS_NOT_KNOWN  (0xFFFF) ///< People count value is not known

/// People count
static count16_t people_count = VALUE_IS_NOT_KNOWN;

/***************************************************************************//**
 * Display people count on display interface.
 ******************************************************************************/
void display_people_count(void)
{
  if (VALUE_IS_NOT_KNOWN == people_count) {
    DI_Print("People count:UNKNOWN", DI_ROW_BUTTON_STATE);
  } else {
    char tmp[21];
    snprintf(tmp, 21, "People count: %5u ", people_count);
    DI_Print(tmp, DI_ROW_BUTTON_STATE);
  }
}

/*******************************************************************************
 * Set the people count value. It could be used to initialize the sensor.
 *
 * @param[in] people_count  People count value to set
 ******************************************************************************/
void set_people_count(count16_t people_count_value)
{
  people_count = people_count_value;
  display_people_count();
}

/*******************************************************************************
 * Get the current people count value measured by sensor.
 *
 * @return Current value of people count.
 ******************************************************************************/
count16_t get_people_count(void)
{
  return people_count;
}

/*******************************************************************************
 * Increase people count value by one. After exceeding the maximum value it set
 * people count to value is not known.
 ******************************************************************************/
void people_count_increase(void)
{
  if (people_count < VALUE_IS_NOT_KNOWN) {
    people_count += 1;
  }
  display_people_count();
}

/*******************************************************************************
 * Decrease people count value by one if value is known and greater than 0.
 ******************************************************************************/
void people_count_decrease(void)
{
  if (people_count > 0 && people_count < VALUE_IS_NOT_KNOWN) {
    people_count -= 1;
  }
  display_people_count();
}

/** @} (end addtogroup PeopleCount) */
/** @} (end addtogroup Sensor) */
