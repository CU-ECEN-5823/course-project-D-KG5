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

/* Bluetooth stack headers */
#include "native_gecko.h"
#include "gatt_db.h"

/* Sensor headers */
#include "sensor.h"
#include "people_count_sensor.h"
#include "em_core.h"

/* Buttons and LEDs headers */
#include "buttons.h"
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

/*******************************************************************************
 * Timer handles defines.
 ******************************************************************************/
//#define TIMER_ID_RESTART            78
//#define TIMER_ID_FACTORY_RESET      77
//#define TIMER_ID_PROVISIONING       66

/** Application timer enumeration. */
typedef enum {
  /* Timer for toggling the the EXTCOMIN signal for the LCD display */
	TIMER_ID_RESTART,
	TIMER_ID_FACTORY_RESET,
	TIMER_ID_PROVISIONING
} swTimer_t;

#define TIMER_CLK_FREQ ((uint32_t)32768) ///< Timer Frequency used
/// Convert miliseconds to timer ticks
#define TIMER_MS_2_TICKS(ms) ((TIMER_CLK_FREQ * (ms)) / 1000)
/// Time equal 0 removes the scheduled timer with the same handle
#define TIMER_REMOVE  0

/// Flag for indicating DFU Reset must be performed
static uint8_t boot_to_dfu = 0;
/// Number of active Bluetooth connections
static uint8_t num_connections = 0;
/// Handle of the last opened LE connection
static uint8_t conn_handle = 0xFF;
/// Flag for indicating that initialization was performed
static uint8_t init_done = 0;

//extern uint8_t adcAvgmapped;
//extern uint32_t adcAvg;

/*******************************************************************************
 * Function prototypes.
 ******************************************************************************/
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *pEvt);

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

  adc_init();
  letimer_init();
  ldma_init();


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
  printf("factory reset\r\n");
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
static void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16_t result;

  // Create unique device name using the last two bytes of the Bluetooth address
  snprintf(name, 20, "sensor server %02x:%02x",
           pAddr->addr[1], pAddr->addr[0]);

  printf("Device name: '%s'\r\n", name);

  result = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name,
                                                       0,
                                                       strlen(name),
                                                       (uint8_t *)name)->result;
  if (result) {
    printf("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n",
           result);
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
  if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0
      || GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
    initiate_factory_reset();
  } else {
	// Initialize Mesh stack in Node operation mode, wait for initialized event
	result = gecko_cmd_mesh_node_init()->result;
	if (result) {
	  snprintf(buf, 30, "init failed (0x%x)", result);
	  DI_Print(buf, DI_ROW_STATUS);
	}
    struct gecko_msg_system_get_bt_address_rsp_t* pAddr =
      gecko_cmd_system_get_bt_address();
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
static void handle_node_initialized_event(
  struct gecko_msg_mesh_node_initialized_evt_t *pEvt)
{
  printf("node initialized\r\n");
  if (pEvt->provisioned) {
    printf("node is provisioned. address:%x, ivi:%ld\r\n",
           pEvt->address,
           pEvt->ivi);

    sensor_node_init();
    enable_button_interrupts();
    DI_Print("provisioned", DI_ROW_STATUS);
  } else {
    printf("node is unprovisioned\r\n");
    DI_Print("unprovisioned", DI_ROW_STATUS);
    printf("starting unprovisioned beaconing...\r\n");
    // Enable ADV and GATT provisioning bearer
    gecko_cmd_mesh_node_start_unprov_beaconing(PB_ADV | PB_GATT);
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
      printf("Started provisioning\r\n");
      DI_Print("provisioning...", DI_ROW_STATUS);
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
      led_init(); /* shared GPIO pins used as LED output */
#endif
      // start timer for blinking LEDs to indicate which node is being provisioned
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TICKS(250),
                                        TIMER_ID_PROVISIONING,
                                        0);
      break;

    case gecko_evt_mesh_node_provisioned_id:
      sensor_node_init();
      printf("node provisioned, got address=%x, ivi:%ld\r\n",
             pEvt->data.evt_mesh_node_provisioned.address,
             pEvt->data.evt_mesh_node_provisioned.iv_index);
      // stop LED blinking when provisioning complete
      gecko_cmd_hardware_set_soft_timer(TIMER_REMOVE, TIMER_ID_PROVISIONING, 0);
      led_set_state(LED_STATE_OFF);
      DI_Print("provisioned", DI_ROW_STATUS);
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
      button_init(); /* shared GPIO pins used as button input */
#endif
      enable_button_interrupts();
      init_done = 1;
      break;

    case gecko_evt_mesh_node_provisioning_failed_id:
      printf("provisioning failed, code %x\r\n",
             pEvt->data.evt_mesh_node_provisioning_failed.result);
      DI_Print("prov failed", DI_ROW_STATUS);
      // start a one-shot timer that will trigger soft reset after small delay
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TICKS(2000),
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
      printf("evt:gecko_evt_le_connection_opened_id\r\n");
      num_connections++;
      conn_handle = pEvt->data.evt_le_connection_opened.connection;
      DI_Print("connected", DI_ROW_CONNECTION);
      break;

    case gecko_evt_le_connection_parameters_id:
      printf("evt:gecko_evt_le_connection_parameters_id: interval %u, latency %u, timeout %u\r\n",
             pEvt->data.evt_le_connection_parameters.interval,
             pEvt->data.evt_le_connection_parameters.latency,
             pEvt->data.evt_le_connection_parameters.timeout);
      break;

    case gecko_evt_le_connection_closed_id:
      // Check if need to boot to dfu mode
      if (boot_to_dfu) {
        // Enter to DFU OTA mode
        gecko_cmd_system_reset(2);
      }
      printf("evt:conn closed, reason 0x%x\r\n",
             pEvt->data.evt_le_connection_closed.reason);
      conn_handle = 0xFF;
      if (num_connections > 0) {
        if (--num_connections == 0) {
          DI_Print("", DI_ROW_CONNECTION);
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
  switch (handle) {
    case TIMER_ID_FACTORY_RESET:
      gecko_cmd_system_reset(0);
      break;

    case TIMER_ID_RESTART:
      gecko_cmd_system_reset(0);
      break;

    case TIMER_ID_PROVISIONING:
      if (!init_done) {
        led_set_state(LED_STATE_PROV);
      }
      break;

    default:
      break;
  }
}

uint8_t map(uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max, uint32_t s){
	return out_min + (s - in_min) * (out_max - out_min) / (in_max - in_min);
}

uint8_t get_adc(){
	adcAvg = (adcBuffer[0] + adcBuffer[1] + adcBuffer[2] + adcBuffer[3]) / ADC_BUFFER_SIZE;
	CORE_irqState_t irq_state = CORE_EnterCritical();
	adcAvgmapped = map(0, 3055, 0, 500, adcAvg);
	CORE_ExitCritical(irq_state);
	printf("ADCAvg: %lu\r\n", adcAvgmapped);
	return adcAvgmapped;
}
/***************************************************************************//**
 *  Handling of external signal events.
 *
 *  @param[in] signal  External signal handle that is serviced by this function.
 ******************************************************************************/
void handle_external_signal_event(uint8_t signal){
	if (signal & EXT_SIGNAL_PB0_PRESS) {
		printf("PB0 pressed\r\n");
		people_count_decrease();
	}
	if (signal & EXT_SIGNAL_PB1_PRESS) {
		printf("PB1 pressed\r\n");
		people_count_increase();
	}
	if(signal & EXT_SIGNAL_CAP_PRESS){
		printf("Cap sensor touched\r\n");
	}
	if(signal & EXT_SIGNAL_LDMA_INT){
//		for(int i = 0; i < ADC_BUFFER_SIZE; i++){
//			printf("ADCBuffer[%d]: %lu\r\n", i, adcBuffer[i]);
//		}
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
      printf("got new %s key with index %x\r\n",
             pEvt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
             pEvt->data.evt_mesh_node_key_added.index);
      break;

    case gecko_evt_mesh_node_model_config_changed_id:
      printf("model config changed\r\n");
      break;

    case gecko_evt_mesh_node_reset_id:
      printf("evt: gecko_evt_mesh_node_reset_id\r\n");
      initiate_factory_reset();
      break;

    case gecko_evt_le_connection_opened_id:
    case gecko_evt_le_connection_parameters_id:
    case gecko_evt_le_connection_closed_id:
      handle_le_connection_events(pEvt);
      break;

    case gecko_evt_gatt_server_user_write_request_id:
      if (gattdb_ota_control
          == pEvt->data.evt_gatt_server_user_write_request.characteristic) {
        enter_to_dfu_ota(pEvt->data.evt_gatt_server_user_write_request.connection);
      }
      break;

    case gecko_evt_hardware_soft_timer_id:
      handle_timer_event(pEvt->data.evt_hardware_soft_timer.handle);
      break;

    case gecko_evt_system_external_signal_id:
      handle_external_signal_event(pEvt->data.evt_system_external_signal.extsignals);
      break;

    default:
      //printf("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
      break;
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
