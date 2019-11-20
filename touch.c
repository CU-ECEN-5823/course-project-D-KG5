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
static PACKSTRUCT(struct button_state {
	uint8_t onoff_current;
	uint8_t onoff_target;
}) button_state;

/**
 * helper function for onoff_update_and_publish
 * set generic state structs and update server
 */
static errorcode_t onoff_update(uint16_t element_index, uint32_t remaining_ms)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = button_state.onoff_current;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = button_state.onoff_target;

  return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        remaining_ms);
}

/**
 * update and publish status of states to network
 */
static errorcode_t onoff_update_and_publish(uint16_t element_index, uint32_t remaining_ms)
{
  errorcode_t e;

  e = onoff_update(element_index, remaining_ms);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_off);
  }

  return e;
}

/**
 * event handler function for subscriber
 * handles requests from client and displays button state on display and update and publish status to network
 */
static void onoff_request(uint16_t model_id, uint16_t element_index, uint16_t client_addr, uint16_t server_addr,
		uint16_t appkey_index, const struct mesh_generic_request *request, uint32_t transition_ms,
		uint16_t delay_ms, uint8_t request_flags){
		printf("ON/OFF request: requested state=<%s>\r\n",
				request->on_off ? "PRESSED" : "RELEASED");

		if(button_state.onoff_current == request->on_off){
//			LOG_DO("NOOP", "INFO");
		} else{
			if(transition_ms == 0){
				printf("Setting button state to <%s>\r\n", request->on_off ? "PRESSED" : "RELEASED");
				button_state.onoff_current = request->on_off;
				button_state.onoff_target = request->on_off;
				if(button_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF){
					DI_Print("Button Released", DI_ROW_PEOPLE_COUNT);
				} else{
					DI_Print("Button Pressed", DI_ROW_PEOPLE_COUNT);
				}
			}
		}
		  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
	//	    onoff_response(element_index, client_addr, appkey_index);
		  }
		  onoff_update_and_publish(element_index, 0);
}

/**
 * change status of button state to current status for subscriber
 */
static void onoff_change(uint16_t model_id, uint16_t element_index, const struct mesh_generic_state *current,
		const struct mesh_generic_state *target, uint32_t remaining_ms){
	if(current->on_off.on != button_state.onoff_current){
		printf("OnOff state changed from %u to %u\r\n", button_state.onoff_current, current->on_off.on);
		button_state.onoff_current = current->on_off.on;
	} else{
		printf("No change in OnOff state\r\n");
	}
}

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
		printf("BUTTON PRESSED\r\n");
	} else{
		req.on_off = MESH_GENERIC_ON_OFF_STATE_OFF;
		printf("BUTTON RELEASED\r\n");
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

/**
 * wrapper function for mesh_lib init and server register handler functions
 * update and publish changes.
 */
void touch_state_init(void){
	_elem_index = 0;

	mesh_lib_init(malloc,free,9);

	mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
			_elem_index,
			onoff_request,
			onoff_change);

	onoff_update_and_publish(_elem_index, 0);
}

