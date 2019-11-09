/***************************************************************************//**
 * @file  sensor.h
 * @brief Sensor header file
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

#ifndef SENSOR_H
#define SENSOR_H

/***************************************************************************//**
 * @defgroup Sensor Sensor Module
 * @brief Sensor Module Implementation
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Sensor
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * Sensor initialization.
 * This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 ******************************************************************************/
void sensor_node_init(void);

/***************************************************************************//**
 *  Handling of mesh sensor server events.
 *  It handles:
 *   - sensor_server_get_request
 *   - sensor_server_get_column_request
 *   - sensor_server_get_series_request
 *   - sensor_setup_server_get_cadence_request
 *   - sensor_setup_server_set_cadence_request
 *   - sensor_setup_server_get_settings_request
 *   - sensor_setup_server_get_setting_request
 *   - sensor_setup_server_set_setting_request
 *
 *  @param[in] pEvt  Pointer to incoming sensor server event.
 ******************************************************************************/
void handle_sensor_server_events(struct gecko_cmd_packet *pEvt);

/** @} (end addtogroup Sensor) */

#endif /* SENSOR_H */
