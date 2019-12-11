/***************************************************************************//**
 * @file  app.c
 * @brief Application code
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

#include <stdlib.h>
#include <stdio.h>
/* Bluetooth stack headers */
#include "native_gecko.h"
#include "gatt_db.h"
#include "gecko_ble_errors.h"
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

/* Sensor headers */
#include "sensor.h"
#include "em_core.h"

/* Buttons and LEDs headers */
#include "buttons.h"
#include "touch.h"
#include "leds.h"
#include "letimer.h"
#include "adc.h"
#include "ldma.h"
#include "app.h"

/* Display Interface header */
#include "display_interface.h"

/* Retarget serial headers */
#include "retargetserial.h"
#include <stdio.h>

/***************************************************************************//**
 * @addtogroup Application
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup app
 * @{
 ******************************************************************************/

/*******************************************************************************
 * Provisioning bearers defines.
 ******************************************************************************/
#define PB_ADV   0x1 ///< Advertising Provisioning Bearer
#define PB_GATT  0x2 ///< GATT Provisioning Bearer

/** Application timer enumeration. */
typedef enum {
  /* Timer for toggling the the EXTCOMIN signal for the LCD display */
	TIMER_ID_RESTART,
	TIMER_ID_FACTORY_RESET,
	TIMER_ID_TOUCH,
	TIMER_ID_PROVISIONING,
	TIMER_ID_BUTTON,
	TIMER_ID_RETRANS,
	TIMER_ID_SAVE_STATE,
	TIMER_ID_NODE_CONFIGURED,
	TIMER_ID_FRIEND_FIND
} swTimer_t;

#define TIMER_CLK_FREQ ((uint32_t)32768) ///< Timer Frequency used
/// Convert miliseconds to timer ticks
#define TIMER_MS_2_TICKS(ms) ((TIMER_CLK_FREQ * (ms)) / 1000)
/// Time equal 0 removes the scheduled timer with the same handle
#define TIMER_REMOVE  0

#define INVALID_ELEMENT_INDEX (uint16_t) 0xFFFFu
#define INVALID_CONN_HANDLE (uint8_t) 0xFFu

/// Flag for indicating DFU Reset must be performed
static uint8_t boot_to_dfu = 0;
/// Number of active Bluetooth connections
static uint8_t num_connections = 0;
/// Handle of the last opened LE connection
uint8_t conn_handle = INVALID_CONN_HANDLE;
/// Flag for indicating that initialization was performed
static uint8_t init_done = 0;

static uint8 lpn_active = 0;

uint8_t button = 0;

uint8_t friend_established = 0;
uint16_t res = 0;
uint8_t request_count = 0;
uint8_t trid = 0; /* transaction id */
uint16_t _elem_index = INVALID_ELEMENT_INDEX;

/*******************************************************************************
 * Function prototypes.
 ******************************************************************************/
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *pEvt);

static uint16_t lpn_state_load(void);
static uint8_t lpn_state_store(void);
static void lpn_state_changed(void);

/***************************************************************************//**
 * Initialise used bgapi classes.
 ******************************************************************************/
void gecko_bgapi_classes_init(void)
{
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  gecko_bgapi_class_gatt_server_init();
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
  gecko_bgapi_class_mesh_node_init();
  gecko_bgapi_class_mesh_proxy_init();
  gecko_bgapi_class_mesh_proxy_server_init();
  gecko_bgapi_class_mesh_sensor_server_init();
  gecko_bgapi_class_mesh_sensor_setup_server_init();
}

/*******************************************************************************
 * Main application code.
 * @param[in] pConfig  Pointer to stack configuration.
 ******************************************************************************/
void appMain(gecko_configuration_t *pConfig)
{
  // Initialize stack
  gecko_stack_init(pConfig);
  gecko_bgapi_classes_init();

  gecko_bgapi_class_mesh_generic_client_init();
  gecko_bgapi_class_mesh_lpn_init();
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();

  // Initialize coexistence interface. Parameters are taken from HAL config.
  gecko_initCoexHAL();

  // Initialize debug prints and display interface
  RETARGET_SerialInit();
  DI_Init();

  // Initialize LEDs and buttons. Note: some radio boards share the same GPIO
  // for button & LED. Initialization is done in this order so that default
  // configuration will be "button" for those radio boards with shared pins.
  // led_init() is called later as needed to (re)initialize the LEDs
  led_init();
  button_init();


  while (1) {
    // Event pointer for handling events
    struct gecko_cmd_packet* evt;

    // If there are no events pending then the next call to gecko_wait_event()
    // may cause device go to deep sleep.
    // Make sure that debug prints are flushed before going to sleep
    if (!gecko_event_pending()) {
      RETARGET_SerialFlush();
    }

    // Check for stack event
    evt = gecko_wait_event();

    bool pass = mesh_bgapi_listener(evt);
    if (pass) {
      handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
    }
  }
}

/***************************************************************************//**
 * This function is called to initiate factory reset. Factory reset may be
 * initiated by keeping one of the pushbuttons pressed during reboot.
 * Factory reset is also performed if it is requested by the provisioner
 * (event gecko_evt_mesh_node_reset_id).
 ******************************************************************************/
static void initiate_factory_reset(void)
{
//  printf("factory reset\r\n");
  DI_Print("\n***\nFACTORY RESET\n***", DI_ROW_STATUS);

  // If connection is open then close it before rebooting
  if (conn_handle != 0xFF) {
    gecko_cmd_le_connection_close(conn_handle);
  }

  // Perform a factory reset by erasing PS storage. This removes all the keys
  // and other settings that have been configured for this node
  gecko_cmd_flash_ps_erase_all();
  // Reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TICKS(2000),
                                    TIMER_ID_FACTORY_RESET,
                                    1);
}

/***************************************************************************//**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] pAddr  Pointer to Bluetooth address.
 ******************************************************************************/
static void set_device_name(bd_addr *pAddr){
  char name[20];
  uint16_t result;

  // Create unique device name using the last two bytes of the Bluetooth address
  snprintf(name, 20, "Search Arm %02x:%02x",
           pAddr->addr[1], pAddr->addr[0]);

//  printf("Device name: '%s'\r\n", name);

  result = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0,
		  strlen(name), (uint8_t *)name)->result;
  if (result) {
//	  printf("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n",
//           result);
  }

  // Show device name on the LCD
  DI_Print(name, DI_ROW_NAME);
}

/***************************************************************************//**
 * Handling of boot event.
 * If needed it performs factory reset. In other case it sets device name
 * and initialize mesh node.
 ******************************************************************************/
static void handle_boot_event(void)
{
  uint16_t result;
  char buf[30];
  // Check pushbutton state at startup.
  // If either PB0 or PB1 is held down then do factory reset
  if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0 || GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
    initiate_factory_reset();
  } else {
	// Initialize Mesh stack in Node operation mode, wait for initialized event
	BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_node_init());
	if (result) {
	  snprintf(buf, 30, "init failed (0x%x)", result);
	  DI_Print(buf, DI_ROW_STATUS);
	}
    struct gecko_msg_system_get_bt_address_rsp_t* pAddr = gecko_cmd_system_get_bt_address();
    set_device_name(&pAddr->address);
  }
}

/***************************************************************************//**
 * Handling of mesh node initialized event.
 * If device is provisioned it initializes the sensor server node.
 * If device is unprovisioned it starts sending Unprovisioned Device Beacons.
 *
 * @param[in] pEvt  Pointer to mesh node initialized event.
 ******************************************************************************/
static void handle_node_initialized_event(struct gecko_msg_mesh_node_initialized_evt_t *pEvt){
//  printf("node initialized\r\n");

  BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_generic_client_init());

  if (pEvt->provisioned) {
//    printf("node is provisioned. address:%x, ivi:%ld\r\n",
//           pEvt->address,
//           pEvt->ivi);

    _elem_index = 0;

    enable_button_interrupts();
    lpn_state_init();
    mesh_lib_init(malloc,free,8);

    lpn_init();

    DI_Print("provisioned", DI_ROW_STATUS);
  } else {
//    printf("node is unprovisioned\r\n");
    DI_Print("unprovisioned", DI_ROW_STATUS);
//    printf("starting unprovisioned beaconing...\r\n");
    // Enable ADV and GATT provisioning bearer
    BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_node_start_unprov_beaconing(PB_ADV | PB_GATT));
  }
}

/***************************************************************************//**
 *  Handling of mesh node provisioning events.
 *  It handles:
 *   - mesh_node_provisioning_started
 *   - mesh_node_provisioned
 *   - mesh_node_provisioning_failed
 *
 *  @param[in] pEvt  Pointer to incoming provisioning event.
 ******************************************************************************/
void handle_node_provisioning_events(struct gecko_cmd_packet *pEvt)
{
  switch (BGLIB_MSG_ID(pEvt->header)) {
    case gecko_evt_mesh_node_provisioning_started_id:
//      printf("Started provisioning\r\n");
      DI_Print("provisioning...", DI_ROW_STATUS);
      // start timer for blinking LEDs to indicate which node is being provisioned
      gecko_cmd_hardware_set_soft_timer(((32768 * 250) / 1000),
                                        TIMER_ID_PROVISIONING,
                                        0);
      break;

    case gecko_evt_mesh_node_provisioned_id:
    	_elem_index = 0;

    	lpn_state_init();

    	mesh_lib_init(malloc,free,8);
    	sensor_node_init();


        res = gecko_cmd_hardware_set_soft_timer(((32768 * 30000) / 1000),
                                                   TIMER_ID_NODE_CONFIGURED,
                                                   1)->result;
        if (res) {
          printf("timer failure?!  %x\r\n", res);
        }
//		printf("node provisioned, got address=%x, ivi:%ld\r\n",
//			 pEvt->data.evt_mesh_node_provisioned.address,
//			 pEvt->data.evt_mesh_node_provisioned.iv_index);
      // stop LED blinking when provisioning complete
		gecko_cmd_hardware_set_soft_timer(TIMER_REMOVE, TIMER_ID_PROVISIONING, 0);
      led_set_state(LED_STATE_OFF);
      DI_Print("provisioned", DI_ROW_STATUS);
      enable_button_interrupts();
      init_done = 1;
      break;

    case gecko_evt_mesh_node_provisioning_failed_id:
//      printf("provisioning failed, code %x\r\n",
//             pEvt->data.evt_mesh_node_provisioning_failed.result);
      DI_Print("prov failed", DI_ROW_STATUS);
      // start a one-shot timer that will trigger soft reset after small delay
      gecko_cmd_hardware_set_soft_timer(((32768 * 2000) / 1000),
                                        TIMER_ID_RESTART,
                                        1);
      break;

    default:
      break;
  }
}

/***************************************************************************//**
 *  Handling of le connection events.
 *  It handles:
 *   - le_connection_opened
 *   - le_connection_parameters
 *   - le_connection_closed
 *
 *  @param[in] pEvt  Pointer to incoming connection event.
 ******************************************************************************/
void handle_le_connection_events(struct gecko_cmd_packet *pEvt)
{
  switch (BGLIB_MSG_ID(pEvt->header)) {
    case gecko_evt_le_connection_opened_id:
//      printf("evt:gecko_evt_le_connection_opened_id\r\n");
      num_connections++;
      conn_handle = pEvt->data.evt_le_connection_opened.connection;
      DI_Print("connected", DI_ROW_CONNECTION);
      lpn_deinit();
      break;

    case gecko_evt_le_connection_parameters_id:
//      printf("evt:gecko_evt_le_connection_parameters_id: interval %u, latency %u, timeout %u\r\n",
//             pEvt->data.evt_le_connection_parameters.interval,
//             pEvt->data.evt_le_connection_parameters.latency,
//             pEvt->data.evt_le_connection_parameters.timeout);
      break;

    case gecko_evt_le_connection_closed_id:
      // Check if need to boot to dfu mode
      if (boot_to_dfu) {
        // Enter to DFU OTA mode
        gecko_cmd_system_reset(2);
      }
//      printf("evt:conn closed, reason 0x%x\r\n",
//             pEvt->data.evt_le_connection_closed.reason);
      conn_handle = 0xFF;
      if (num_connections > 0) {
        if (--num_connections == 0) {
          DI_Print("", DI_ROW_CONNECTION);
          lpn_init();
        }
      }
      break;

    default:
      break;
  }
}

/***************************************************************************//**
 *  Entering to OTA DFU.
 *
 *  @param[in] connection  Connection that handle OTA DFU.
 ******************************************************************************/
void enter_to_dfu_ota(uint8_t connection)
{
  // Set flag to enter to OTA mode
  boot_to_dfu = 1;
  // Send response to Write Request
  gecko_cmd_gatt_server_send_user_write_response(connection,
                                                 gattdb_ota_control,
                                                 bg_err_success);
  // Close connection to enter to DFU OTA mode
  gecko_cmd_le_connection_close(connection);
}

/***************************************************************************//**
 *  Handling of timer events.
 *
 *  @param[in] handle  Timer handle that is serviced by this function.
 ******************************************************************************/
void handle_timer_event(uint8_t handle)
{
//	CORE_DECLARE_IRQ_STATE;
  switch (handle) {
    case TIMER_ID_FACTORY_RESET:
      gecko_cmd_system_reset(0);
      break;

    case TIMER_ID_RESTART:
      gecko_cmd_system_reset(0);
      break;

    case TIMER_ID_TOUCH:
    	break;

    case TIMER_ID_PROVISIONING:
      if (!init_done) {
        led_set_state(LED_STATE_PROV);
      }
      break;

	case TIMER_ID_RETRANS:
		send_onoff_request(1);   /* 1 indicates that this is a retransmission */
		/* stop retransmission timer if it was the last attempt */
		if (request_count == 0) {
			gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_RETRANS, false);
		}
		break;

	case TIMER_ID_SAVE_STATE:
		lpn_state_store();
		break;

	case TIMER_ID_NODE_CONFIGURED:
		if(!lpn_active){
//			printf("trying to initialize lpn...\r\n");
			lpn_init();
		}
		break;

	case TIMER_ID_FRIEND_FIND:
//		printf("trying to find friend...\r\n");
		res = gecko_cmd_mesh_lpn_establish_friendship(0)->result;
		if(res != 0){
			printf("Ret code 0x%x\r\n", res);
		}
		break;

    default:
      break;
  }
}

/**
 * Initialize LPN functionality
 */
void lpn_init(void){
	uint16 result;

	// Do not initialize LPN if lpn is currently active
	// or any GATT connection is opened
	if (lpn_active || num_connections) {
		return;
	}

	// Initialize LPN functionality.
	result = gecko_cmd_mesh_lpn_init()->result;
	if (result) {
//		printf("LPN init failed (0x%x)\r\n", result);
		return;
	}
	lpn_active = 1;
//	printf("LPN initialized\r\n");
	DI_Print("LPN on", DI_ROW_LPN);

	// Configure the lpn with following parameters:
	// - Minimum friend queue length = 2
	// - Poll timeout = 5 seconds
	// - Retry interval = 0 ms
	result = gecko_cmd_mesh_lpn_configure(2, 5 * 1000)->result;
	if (result) {
//		printf("LPN conf failed (0x%x)\r\n", result);
		return;
	}

//	printf("trying to find friend...\r\n");
	result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

	if (result != 0) {
		printf("ret.code %x\r\n", result);
	}
}

/**
 * Deinitialize LPN functionality
 */
void lpn_deinit(void){
	uint16_t res;

	if (!lpn_active) {
		return; // lpn feature is currently inactive
	}

	res = gecko_cmd_hardware_set_soft_timer(0, // cancel friend finding timer
											 TIMER_ID_FRIEND_FIND,
											 1)->result;

	// Terminate friendship if exist
	res = gecko_cmd_mesh_lpn_terminate_friendship()->result;
	if (res) {
//		printf("Friendship termination failed (0x%x)\r\n", res);
	}
	// turn off lpn feature
	res = gecko_cmd_mesh_lpn_deinit()->result;
	if (res) {
//		printf("LPN deinit failed (0x%x)\r\n", res);
	}
	lpn_active = 0;
//	printf("LPN deinitialized\r\n");
	DI_Print("LPN off", DI_ROW_LPN);
}

/**
 * Initialize data memory struct for lpn node 1
 */
void lpn_state_init(void){
	memset(&lpn_state, 0, sizeof(struct lpn_state));
	uint16_t res = lpn_state_load();
	if (res == -1) {
//		printf("lpn_state_load(): size of lpn_state has changed, using defaults\r\n");
	} else if(res == 0){
//		printf("lpn_state_load(): success\r\n");
	} else{
		printf("lpn_state_load(): failed with error 0x%x, using defaults\r\n", res);
	}
	printf("********Persistent Data**********\r\n");
	printf("Old Current ADC: %u\r\n", lpn_state.adc_current);
	printf("Old Previous ADC: %u\r\n", lpn_state.adc_previous);
	printf("*********************************\r\n");
	if(lpn_active){
		lpn_state_changed();
	}
}

/***************************************************************************//**
 * This function loads the saved lpn state from Persistent Storage and
 * copies the data in the global variable lpn_state.
 * If PS key with ID 0x4004 does not exist or loading failed,
 * lpn_state is set to zero and some default values are written to it.
 *
 * @return 0 if loading succeeds. error code if loading fails. -1 if lpn_state changed
 ******************************************************************************/
static uint16_t lpn_state_load(void){
	struct gecko_msg_flash_ps_load_rsp_t* pLoad;

	pLoad = gecko_cmd_flash_ps_load(0x4008);

	// Set default values if ps_load fail or size of lpn_state has changed
	if (pLoad->result || (pLoad->value.len != sizeof(struct lpn_state))) {
		memset(&lpn_state, 0, sizeof(struct lpn_state));
		lpn_state.onoff_current = 0x00;
		lpn_state.onoff_target = 0x01;
		lpn_state.adc_default = 250;
		lpn_state.adc_min = 0;
		lpn_state.adc_max = 500;
		printf("Defaults used\r\n");

		if(pLoad->result != 0){
			return pLoad->result;
		}

		return -1;
	}

	memcpy(&lpn_state, pLoad->value.data, pLoad->value.len);

	return 0;
}


/***************************************************************************//**
 * This function saves the current lpn state in Persistent Storage so that
 * the data is preserved over reboots and power cycles.
 * The light state is hold in a global variable lightbulb_state.
 * A PS key with ID 0x4004 is used to store the whole struct.
 *
 * @return 0 if saving succeed
 ******************************************************************************/
static uint8_t lpn_state_store(void){
	BTSTACK_CHECK_RESPONSE(gecko_cmd_flash_ps_save(0x4008, sizeof(struct lpn_state), (const uint8*)&lpn_state));
//	printf("LPN State Stored\r\n");

	return 0;
}

/***************************************************************************//**
 * This function is called each time the lpn state in RAM is changed.
 * It sets up a soft timer that will save the state in flash after small delay.
 * The purpose is to reduce amount of unnecessary flash writes.
 ******************************************************************************/
static void lpn_state_changed(void){
	gecko_cmd_hardware_set_soft_timer(((32768 * 500) / 1000), TIMER_ID_SAVE_STATE, 1);
//	printf("LPN State Changed\r\n");
}

/**
 * function to display button state values to terminal and LCD
 */
void display_button(void){
	if(lpn_state.onoff_current == 0xFFu){
		DI_Print("Button State: UNKNOWN", DI_ROW_BUTTON_STATE);
//		printf("Button State: UNKNOWN\r\n");
	} else{
		char button_disp[24];
		snprintf(button_disp, 24, "Button State: %s", lpn_state.onoff_current ? "PRESS" : "RELEASE");
		DI_Print(button_disp, DI_ROW_BUTTON_STATE);
//		printf("Button State: %s\r\n", lpn_state.onoff_current ? "PRESS" : "RELEASE");
	}
}

/**
 * helper function for get_adc to display adc values to terminal and LCD
 */
void display_adc(void){
	if(lpn_state.adc_current == 0xFFFFu){
		DI_Print("ADC: UNKNOWN", DI_ROW_ADC);
//		printf("ADC: UNKNOWN\r\n");
	} else{
		char tmp[21];
		snprintf(tmp, 21, "Muscle Activity: %3u", lpn_state.adc_current);
		DI_Print(tmp, DI_ROW_ADC);
//		printf("Current ADC: %u\r\n", lpn_state.adc_current);
//		printf("Previous ADC: %u\r\n", lpn_state.adc_previous);
	}
}

/**
 * map adc values for linear calibration
 */
uint16_t map(uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max, uint32_t s){
	return out_min + (s - in_min) * (out_max - out_min) / (in_max - in_min);
}

/**
 * get adc buffer, then average and map it for calibration
 */
uint16_t get_adc(){
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	adcAvg = (uint16_t)((adcBuffer[0] + adcBuffer[1] + adcBuffer[2] + adcBuffer[3]) / ADC_BUFFER_SIZE);

	lpn_state.adc_previous = lpn_state.adc_current;
	lpn_state.adc_current = map(0, 3055, 0, 500, adcAvg);
	CORE_EXIT_CRITICAL();

	display_adc();
	if(lpn_active){
		lpn_state_changed();
	}

	return lpn_state.adc_current;
}
/***************************************************************************//**
 *  Handling of external signal events.
 *
 *  @param[in] signal  External signal handle that is serviced by this function.
 ******************************************************************************/
void handle_external_signal_event(uint8_t signal){
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	request_count = 3;
	CORE_EXIT_CRITICAL();
//	printf("Signal is %x\r\n", signal);
	if (signal & EXT_SIGNAL_FRIENDSHIP_EST) {
		if(friend_established == 0){
//			printf("Friendship established\r\n");
			adc_init();
			letimer_init();
			ldma_init();
			sensor_node_init();
			friend_established = 1;
		}

	}
	if (signal & EXT_SIGNAL_PB1_PRESS) {
//		printf("PB1 pressed\r\n");
	}
	if(signal & EXT_SIGNAL_CAP_PRESS){
//		printf("Cap sensor pressed\r\n");
		CORE_DECLARE_IRQ_STATE;
		CORE_ENTER_CRITICAL();
		lpn_state.onoff_current = 1; /* set global var to 1 if button is pressed */
		lpn_state.onoff_target = 0;
		button = 1;
		CORE_EXIT_CRITICAL();

		if(lpn_active){
			lpn_state_changed();
		}

		if(button == 1){
			send_onoff_request(0); /* 0 indicates that this is an original transmission */
			/* start a repeating soft timer to trigger retransmission of the request after 50 ms delay */
			gecko_cmd_hardware_set_soft_timer(((32768 * 50) / 1000), TIMER_ID_RETRANS, false);
			button = 0;
		}
	}
	if(signal & EXT_SIGNAL_CAP_RELEASE){
//		printf("Cap sensor released\r\n");
		CORE_ENTER_CRITICAL();
		lpn_state.onoff_current = 0; /* set global var to 0 if button is released */
		lpn_state.onoff_target = 1;
		CORE_EXIT_CRITICAL();

		if(lpn_active){
			lpn_state_changed();
		}

		if(button == 0){
			send_onoff_request(0); /* 0 indicates that this is an original transmission */
			/* start a repeating soft timer to trigger retransmission of the request after 50 ms delay */
			gecko_cmd_hardware_set_soft_timer(((32768 * 50) / 1000), TIMER_ID_RETRANS, false);
			button = 1;
		}

	}
	if(signal & EXT_SIGNAL_LDMA_INT){
		get_adc();
	}
}

/***************************************************************************//**
 * Handling of stack events. Both BLuetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt_id  Incoming event ID.
 * @param[in] pEvt    Pointer to incoming event.
 ******************************************************************************/
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *pEvt)
{
  if (NULL == pEvt) {
    return;
  }

  // Handle events
  switch (evt_id) {
    case gecko_evt_system_boot_id:
      handle_boot_event();
      break;

    case gecko_evt_mesh_node_initialized_id:
      handle_node_initialized_event(&(pEvt->data.evt_mesh_node_initialized));
      break;

    case gecko_evt_mesh_node_provisioning_started_id:
    case gecko_evt_mesh_node_provisioned_id:
    case gecko_evt_mesh_node_provisioning_failed_id:
      handle_node_provisioning_events(pEvt);
      break;

    case gecko_evt_mesh_sensor_server_get_request_id:
    case gecko_evt_mesh_sensor_server_get_column_request_id:
    case gecko_evt_mesh_sensor_server_get_series_request_id:
    case gecko_evt_mesh_sensor_server_publish_id:
    case gecko_evt_mesh_sensor_setup_server_get_cadence_request_id:
    case gecko_evt_mesh_sensor_setup_server_set_cadence_request_id:
    case gecko_evt_mesh_sensor_setup_server_get_settings_request_id:
    case gecko_evt_mesh_sensor_setup_server_get_setting_request_id:
    case gecko_evt_mesh_sensor_setup_server_set_setting_request_id:
      handle_sensor_server_events(pEvt);
      break;

    case gecko_evt_mesh_node_key_added_id:
//		printf("got new %s key with index %x\r\n",
//			 pEvt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
//			 pEvt->data.evt_mesh_node_key_added.index);

		res = gecko_cmd_hardware_set_soft_timer(((32768 * 5000) / 1000),
												 TIMER_ID_NODE_CONFIGURED,
												 1)->result;
		if (res) {
			printf("timer failure  0x%x\r\n", res);
		}
		break;

    case gecko_evt_mesh_node_model_config_changed_id:
//		printf("model config changed\r\n");
		res = gecko_cmd_hardware_set_soft_timer(((32768 * 5000) / 1000),
												 TIMER_ID_NODE_CONFIGURED,
												 1)->result;
		if (res) {
			printf("timer failure  0x%x\r\n", res);
		}
		break;

    case gecko_evt_mesh_node_config_set_id:
//		printf("model config set\r\n");
		// try to init lpn 5 seconds after configuration set
		res = gecko_cmd_hardware_set_soft_timer(((32768 * 5000) / 1000),
												 TIMER_ID_NODE_CONFIGURED,
												 1)->result;
		if (res) {
			printf("timer failure  %x\r\n", res);
		}
		break;

    case gecko_evt_mesh_node_reset_id:
//      printf("evt: gecko_evt_mesh_node_reset_id\r\n");
      initiate_factory_reset();
      break;

    case gecko_evt_le_connection_opened_id:
    case gecko_evt_le_connection_parameters_id:
    case gecko_evt_le_connection_closed_id:
      handle_le_connection_events(pEvt);
      break;

    case gecko_evt_gatt_server_user_write_request_id:
      if (gattdb_ota_control == pEvt->data.evt_gatt_server_user_write_request.characteristic) {
        enter_to_dfu_ota(pEvt->data.evt_gatt_server_user_write_request.connection);
      }
      break;

    case gecko_evt_hardware_soft_timer_id:
      handle_timer_event(pEvt->data.evt_hardware_soft_timer.handle);
      break;

    case gecko_evt_system_external_signal_id:
      handle_external_signal_event(pEvt->data.evt_system_external_signal.extsignals);
      break;

    case gecko_evt_mesh_lpn_friendship_established_id:
//      printf("friendship established\r\n");
    	gecko_external_signal(EXT_SIGNAL_FRIENDSHIP_EST);
      DI_Print("LPN with friend", DI_ROW_LPN);
      break;

    case gecko_evt_mesh_lpn_friendship_failed_id:
//      printf("friendship failed\r\n");
      DI_Print("no friend", DI_ROW_LPN);
      // try again in 2 seconds
      res = gecko_cmd_hardware_set_soft_timer(((32768 * 2000) / 1000),
                                                 TIMER_ID_FRIEND_FIND,
                                                 1)->result;
      if (res) {
        printf("timer failure  %x\r\n", res);
      }
      break;

    case gecko_evt_mesh_lpn_friendship_terminated_id:
//      printf("friendship terminated\r\n");
      DI_Print("friend lost", DI_ROW_LPN);
      if (num_connections == 0) {
        // try again in 2 seconds
        res = gecko_cmd_hardware_set_soft_timer(((32768 * 2000) / 1000),
                                                   TIMER_ID_FRIEND_FIND,
                                                   1)->result;
        if (res) {
          printf("timer failure  %x\r\n", res);
        }
      }
      break;


    default:
      //printf("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
      break;
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
