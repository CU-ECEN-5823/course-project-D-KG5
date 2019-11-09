/***************************************************************************//**
 * @file  temperature_sensor.c
 * @brief Temperature sensor implementation
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
#include "temperature_sensor.h"
#include "display_interface.h"
#include "si7013.h"

/***************************************************************************//**
 * @addtogroup Sensor
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup TemperatureSensor
 * @{
 ******************************************************************************/

#define VALUE_IS_NOT_KNOWN  (0xFF) ///< Temperature value is not known

/// Temperature
static temperature_8_t temperature = VALUE_IS_NOT_KNOWN;

/***************************************************************************//**
 * Display temperature on display interface.
 ******************************************************************************/
void display_temperature(void)
{
  if ((temperature_8_t)VALUE_IS_NOT_KNOWN == temperature) {
    DI_Print("Temperature: UNKNOWN", DI_ROW_TEMPERATURE);
  } else {
    char tmp[21];
    snprintf(tmp, 21, "Temperature: %3d.%1dC ", temperature / 2, (temperature * 5) % 10);
    DI_Print(tmp, DI_ROW_TEMPERATURE);
  }
}

/*******************************************************************************
 * Initialize the temperature sensor.
 ******************************************************************************/
void init_temperature_sensor()
{
  bool status = Si7013_Detect(I2C0, SI7021_ADDR, NULL);
  if (!status) {
    printf("I2C Error\r\n");
  }
  get_temperature();
}

/*******************************************************************************
 * Get the current temperature value measured by sensor.
 *
 * @return Current value of temperature.
 ******************************************************************************/
temperature_8_t get_temperature(void)
{
  int32_t tempData = 0;
#ifndef FEATURE_I2C_SENSOR
  static int32_t DummyValue = 0l;

  /* Use dummy counter for boards that does not support I2C sensor feature */
  tempData = DummyValue + 20000l;
  DummyValue = (DummyValue + 1000l) % 21000l;
  tempData = (((tempData * 2) + 499) / 1000);
  temperature = (temperature_8_t)tempData;
#else
  uint32_t tempRH = 0;

  /* Sensor relative humidity and temperature measurement returns 0 on success, nonzero otherwise */
  if (Si7013_MeasureRHAndTemp(I2C0, SI7021_ADDR, &tempRH, &tempData) != 0) {
    temperature = VALUE_IS_NOT_KNOWN;
  } else if ((tempData > 63500) || (tempData < -64000)) {
    temperature = VALUE_IS_NOT_KNOWN;
  } else {
    tempData = (((tempData * 2) + 499) / 1000);
    temperature = (temperature_8_t)tempData;
  }
#endif

  display_temperature();
  return temperature;
}

/** @} (end addtogroup TemperatureSensor) */
/** @} (end addtogroup Sensor) */
