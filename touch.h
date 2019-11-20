/*
 * ble.h
 *
 *  Created on: Oct 1, 2019
 *      Author: Dhruva
 */

#ifndef SRC_BTMESH_H_
#define SRC_BTMESH_H_
#include <stdint.h>
#include "native_gecko.h"
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"

/* global var for connection handle */
extern uint8_t conn_handle;
extern uint8_t request_count;
extern uint16_t _elem_index;
extern uint8_t trid;

void send_onoff_request(uint8_t retrans);
void touch_state_init(void);

#endif /* SRC_BTMESH_H_ */
