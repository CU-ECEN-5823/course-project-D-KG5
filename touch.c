/*
 * ble.c
 *
 *  Created on: Oct 1, 2019
 *      Author: Dhruva
 */

 /*******************************************************************************
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

/* Heavy inspiration from the soc-btmesh-light and soc-btmesh-switch example projects */

/* standard library headers */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <touch.h>

/* BG stack headers */
#include "bg_types.h"
#include "gatt_db.h"
#include "native_gecko.h"
#include "gecko_ble_errors.h"
#include "display_interface.h"
#include "mesh_lib.h"


#include "gecko_ble_errors.h"
#include "em_core.h"
#include "display.h"

extern uint8_t buttonValue;

/**
 * button_state struct containing current and target onoff states
 */
//static PACKSTRUCT(struct button_state {
//	uint8_t onoff_current;
//	uint8_t onoff_target;
//}) button_state;

/**
 * send onoff request based on button state to client
 */
void send_onoff_request(uint8_t retrans){
	uint16 resp;
	uint16 delay;
	struct mesh_generic_request req;
	const uint32 transtime = 0; /* using zero transition time by default */

	req.kind = mesh_generic_request_on_off;
	if(buttonValue){
		req.on_off = MESH_GENERIC_ON_OFF_STATE_ON;
//		printf("BUTTON PRESSED\r\n");
	} else{
		req.on_off = MESH_GENERIC_ON_OFF_STATE_OFF;
//		printf("BUTTON RELEASED\r\n");
	}
	// increment transaction ID for each request, unless it's a retransmission
	if (retrans == 0){
		trid++;
	}
	/* delay for the request that decrements in 50ms increments
	*/
	delay = (request_count - 1) * 50;
	resp = mesh_lib_generic_client_publish(MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
			_elem_index, trid, &req, transtime, delay, 0);

	if(resp){
		printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
	} else {
		printf("Request sent, trid = %u, delay = %dms\r\n", trid, delay);
	}

	/* keep track of how many requests has been sent */
	if(request_count > 0){
		request_count--;
	}
}
