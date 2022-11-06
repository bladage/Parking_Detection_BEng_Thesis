/**
 * @brief Sending the RADAR chip's data and the MMC's data via Bluetooth Low Energy (BLE) to the Raspberry Pi.
 * This device uses the peripheral role (p role; server) and the Raspberry Pi the central role (c roll; client)
 *
 * @author Bennet Ladage
 * @date 2022-07-10
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 */

#include "pd_project_includes.h"  // includes for XM122, BLE, MMC and everything else...
#include "pd_project_defines.h"   // defines for XM122, RADAR, MMC, etc; BLE defines in this main

// Defines for BLE
#define APP_BLE_CONN_CFG_TAG    1
#define APP_BLE_OBSERVER_PRIO   3

// BLE's GAP parameters: PPCP (Periperal Preferred Connection Parameters)
#define DEVICE_NAME             "PD_Sensor"
#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN        /**< UUID type for the Nordic UART Service (vendor specific). */
#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(20, UNIT_1_25_MS)  // time where peripheral has to answer central that he is still alive
#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(75, UNIT_1_25_MS)  
#define SLAVE_LATENCY           0                                 // how often the peripheral (this device) skips the connection event/interval
#define CONN_SUP_TIMEOUT        MSEC_TO_UNITS(10000, UNIT_10_MS)   // time to wait until the link is lost after a connection event, i.e., this devices falls back to disconneced state

// BLE's advertising parameters:
#define APP_ADV_INTERVAL    64         // 40*0.625ms = 25ms     
#define APP_ADV_DURATION    0 //18000      // 5 ms time period for broadcasts

// BLE's PPCP (Peripheral Preferred Connection Parameters)
#define FIRST_CONN_PARMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)   
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define DEAD_BEEF           0xDEADBEEF

// Message types for the master device (BLE's central role, raspi)
#define MSG_SEND_SIGNALS      'a'
#define MSG_SEND_META_DATA    'b'
#define MSG_SEND_RADAR_SIG    'c'
#define MSG_SEND_RADAR_META   'd'
#define MSG_SEND_MMC_SIG      'e'
#define MSG_SEND_MMC_META     'f'
#define MSG_PARKING_SPOT_OCCU 'g'   // parking space is occupied
#define MSG_PARKING_SPOT_VACA 'h'   // parking space is vacant


// Global variables for BLE
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);   // Nordic UART Service (NUS) instance
NRF_BLE_QWR_DEF(m_qwr);                             // queued writer instance; QWRS if connection with n>1 devices!
NRF_BLE_GATT_DEF(m_gatt);                           // define instance for GATT
BLE_ADVERTISING_DEF(m_advertising);                 // instance for advertising

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; // connection handle for BLE
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


// Global variables for parking detection
bool parking_spot_is_occupied = false;
float uncertainty_pd = 95;  // in %; is this information really necessary here? Or, raspi (master) only???? TODO
uint16_t sleep_interval_pd = 100; // interval in UNIT(?) during the parking detection is "off-line"

uint16_t number_of_measurements = 100;          // will be set by master
uint16_t delay_between_measurements_ms = 10;    // "
uint16_t sig_len_radar = 10;                    // meta data of the sensors
uint16_t sig_len_mmc = 10;                      // "


/*
  Acconeer's functions of the main-template
*/
int _write(int file, char *ptr, int len)
{
  static char log_buffer[256];
  static int  log_index;

  for (int i = 0; i < len; i++)
  {
    log_buffer[log_index] = *ptr++;
    if (log_buffer[log_index] == '\n' || (log_index == (sizeof(log_buffer) - 2)))
    {
      if (log_buffer[log_index] != '\n')
      {
        log_index++;
      }

      log_buffer[log_index] = 0;
      // The additional "\n" is needed when running from SEGGER Embedded Studio
      NRF_LOG_INFO("%s\n", NRF_LOG_PUSH(log_buffer));
      log_index = 0;
    }
    else
    {
      log_index++;
    }
  }

  return len;
}


extern void *_sbrk(int incr);


extern void *__HeapBase;
extern void *__HeapLimit;


// The built in version for newlib of _sbrk() does not check
// the heap for overflow so we need our own version of it
void *_sbrk(int incr)
{
  static void *heap = NULL;
  void        *prev_heap;
  void        *next_heap;

  if (heap == NULL)
  {
    heap = (void * )&__HeapBase;
  }

  prev_heap = heap;
  next_heap = (void *)((unsigned int)heap + incr);

  if (next_heap >= (void *)&__HeapLimit)
  {
    return (void *)-1;
  }
  else
  {
    heap = next_heap;
    return prev_heap;
  }
}

/*
  Acconeer's functions: low power envelope, modified by Bennet
*/

//! @brief setup envelope service
static acc_service_handle_t create_envelope_service(void)
{
  bool                 success        = true;
  acc_service_handle_t service_handle = NULL;

  float range_start_m  = RANGE_START;
  float range_length_m = RANGE_LENGTH;

  acc_service_configuration_t envelope_configuration = acc_service_envelope_configuration_create();

  if (envelope_configuration == NULL)
  {
    printf("Could not create envelope configuration\n");
    success = false;
  }

  if (success)
  {
    acc_service_requested_start_set(envelope_configuration, range_start_m);
    acc_service_requested_length_set(envelope_configuration, range_length_m);
    acc_service_power_save_mode_set(envelope_configuration, SELECTED_POWER_SAVE_MODE);
    acc_service_profile_set(envelope_configuration, SELECTED_SERVICE_PROILE);
    acc_service_hw_accelerated_average_samples_set(envelope_configuration, HWAAS);
    acc_service_envelope_downsampling_factor_set(envelope_configuration, DOWNSAMPLING_FACTOR);

    //acc_service_mur_set(envelope_configuration, ACC_SERVICE_MUR_9); // change MUR=17.3 m instead of 11.5 m
                

    service_handle = acc_service_create(envelope_configuration);

    if (service_handle == NULL)
    {
      printf("Could not create envelope service\n");
      success = false;
    }
  }

  acc_service_envelope_configuration_destroy(&envelope_configuration);

  return success ? service_handle : NULL;
}

/*
  Misc
*/
//! @brief Toggle the LED
static void led_blinking(uint8_t iterations, uint8_t delay_ms)
{
  for(uint8_t idx=0; idx < iterations; idx++)
  {
    nrf_gpio_pin_toggle(LED_2);
    nrf_delay_ms(delay_ms);
  }
}

/*
  BLE Functions
*/

//! @brief GAP initialization
static void gap_params_init()
{
  ret_code_t err_code;

  ble_gap_conn_params_t gap_conn_params;  // communication parameters
  ble_gap_conn_sec_mode_t sec_mode;       // security mode

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode); // no security, e.g., MITM=man in the middle is another option
  
  err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME)); // set device name
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params)); // clear the memory of the connection parameters
  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params); // set all the PPCPs
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);

}

//! @brief GATT initialization
static void gatt_init()
{
  ret_code_t err_code;
  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  // new:
  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

// TODO maybe: replace timer event and send the radar & mmc data
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
        break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
        break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
        break;
        
        default:
        break;
    }
}

static void uart_init(void)
{
  uint32_t err_code_UART;                        // error value for UART communication
  const app_uart_comm_params_t config_UART =     // struct to hold the uart configurations
  {
    UART_RX, 
    UART_TX,
    UART_RTS,
    UART_CTS,
    APP_UART_FLOW_CONTROL_DISABLED, // hardware flow control disabled
    false, // parity = none
    #if defined (UART_PRESENT)
    NRF_UART_BAUDRATE_115200
    #else
    NRF_UARTE_BAUDRATE_115200
    #endif
  };

  // Initialize the UART module:
  APP_UART_FIFO_INIT(&config_UART, 
  UART_RX_BUFF_SIZE, 
  UART_TX_BUFF_SIZE, 
  uart_event_handle, 
  APP_IRQ_PRIORITY_LOWEST, 
  err_code_UART);
  NRF_LOG_INFO("Error Code UART: %d", err_code_UART);
  APP_ERROR_CHECK(err_code_UART); // check if everything initialized correctly
}

// /new

//! @brief Error handler for queue writer */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

//! @brief Handle NUS events, i.e., receiving data; TODO: set global variable when the cars parks here; decision is taken by the master (raspi)
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA) // What if the device receives data?
    {
        uint32_t err_code;
        //uint8_t rx_data[BLE_NUS_MAX_DATA_LEN];
        //memcpy(rx_data, p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        
        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for(uint32_t i = 0; i < p_evt->params.rx_data.length; i++) // Print rx data via uart
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }

        // respond to the message of the rx data
        uint8_t * nus_array = NULL;                     // array which is send via NUS
        uint16_t nus_msg_len = BLE_NUS_MAX_DATA_LEN;    // maximum message size, should be 20 byte
        uint16_t number_of_nus_msgs = 0;                // How many nus messages have to be send that the whole data array is transmitted?
        uint16_t k;                                     // loop index
        
        // send all signals, i.e., radar signal, magnetic sensor
        if(p_evt->params.rx_data.p_data[0] == MSG_SEND_SIGNALS)
        //if(rx_data[0] == MSG_SEND_SIGNALS)
        {
           // create pseudo signal; TODO extracting radar + mmc signal   
            uint16_t signal_len = 2*BLE_NUS_MAX_DATA_LEN+3; // signal length, i.e., radar sig + mmc sig
            uint8_t data_array[signal_len];                 // radar sig + mmc sig
            
            uint8_t offset = 0;
            for(k = 0; k < signal_len; k++)
            {
              if(k > BLE_NUS_MAX_DATA_LEN)
              {
                offset = BLE_NUS_MAX_DATA_LEN;
              }             
              data_array[k] = 65 + k - offset; //rx_data[1];  //p_evt->params.rx_data.p_data[1];
            }
            
            // calculate how often a 20 bytes message has to be send & zero padding if the signal length isn't divideable by 20
            if(signal_len % BLE_NUS_MAX_DATA_LEN == 0)
            {
              number_of_nus_msgs = signal_len / BLE_NUS_MAX_DATA_LEN;
              nus_array = data_array;
            }
            else
            {              
              number_of_nus_msgs = (signal_len / BLE_NUS_MAX_DATA_LEN) + 1;       // round: ceil()
              uint8_t nus_array_len = BLE_NUS_MAX_DATA_LEN * number_of_nus_msgs;
              nus_array = calloc(nus_array_len, sizeof(uint8_t));                 // allocate memory and set every element to 0

              for(k = 0; k < signal_len; k++) // zero padding for the message
              {
                nus_array[k] = data_array[k];
              }           
            }
            
            // send signal, i.e., number_of_nus_mgs x 20 byte messages in a row
            for(k = 0; k < number_of_nus_msgs; k++) 
            { 
              do
              {
                err_code = ble_nus_data_send(&m_nus, &nus_array[k*BLE_NUS_MAX_DATA_LEN], &nus_msg_len, m_conn_handle);
              } while (err_code == NRF_ERROR_RESOURCES);
            }

            if(signal_len % BLE_NUS_MAX_DATA_LEN != 0) // just if calloc was used before
            {
              free(nus_array);
            }
          }

          // send meta data of the radar sensor and the magnetic sensor
          else if(p_evt->params.rx_data.p_data[0] == MSG_SEND_META_DATA)
          {
          }

          // send the radar signal only
          else if(p_evt->params.rx_data.p_data[0] == MSG_SEND_RADAR_SIG)
          {
          }

          // send the radar's meta data only 
          else if(p_evt->params.rx_data.p_data[0] == MSG_SEND_RADAR_META)
          {
          }

          // send the magnetic sensor's signals only
          else if(p_evt->params.rx_data.p_data[0] == MSG_SEND_MMC_SIG)
          {
          }

          // send the magnetic sensor's meta data only 
          else if(p_evt->params.rx_data.p_data[0] == MSG_SEND_MMC_META)
          {
          }

          // master estimated that the parking space is occupied
          else if(p_evt->params.rx_data.p_data[0] == MSG_PARKING_SPOT_OCCU)
          {
            parking_spot_is_occupied = true;
            // TODO: further actions?
          }

          // master estimated that the parking space is vacant
          else if(p_evt->params.rx_data.p_data[0] == MSG_PARKING_SPOT_VACA)
          {
            parking_spot_is_occupied = false;
            // TODO: further actions?
          }

          // case if invalid command was received
          else
          {
            uint8_t data_array_err[BLE_NUS_MAX_DATA_LEN] = {'I', 'n', 'v', 'a', 'l', 'i', 'd', ' ', 'M', 'S', 'G', '!', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
            do
            {
              err_code = ble_nus_data_send(&m_nus, data_array_err, &nus_msg_len, m_conn_handle);
            } while (err_code == NRF_ERROR_RESOURCES);            
          }
    }
}


//! @brief Services initialization
static void services_init(void)
{
  ret_code_t err_code;
  ble_nus_init_t nus_init;
  nrf_ble_qwr_init_t qwr_init = {0};                // struct for the queue writer (qwt) instace

  qwr_init.error_handler = nrf_qwr_error_handler;   // error hander qwt, e.g., connection lost a data transmission faild hence  

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);   // start qwt
  APP_ERROR_CHECK(err_code);

  // Initialize NUS:
  memset(&nus_init, 0, sizeof(nus_init));
  nus_init.data_handler = nus_data_handler;

  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);
}

//! @brief event handler for connection parameters update
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
  ret_code_t err_code;
  if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) // update denied by master
  {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE); // disconnect if update fails, TODO: contradict with cp_init.disconnect_on_fail=false???
    APP_ERROR_CHECK(err_code);
  }
  // else
  //if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED) // update accepted by master
  //{

  //}
}

//! @brief error event handler for connection parameters update
static void conn_params_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

//! @brief PPCP (peripheral preferred connection parameters) initialization
static void conn_params_init(void)
{
  ret_code_t err_code;
  ble_conn_params_init_t cp_init;       // structur for PPCP
  memset(&cp_init, 0, sizeof(cp_init)); // clear memory  

  cp_init.p_conn_params                   = NULL;                           // pointer
  cp_init.first_conn_params_update_delay  = FIRST_CONN_PARMS_UPDATE_DELAY;  // after this delay the first request is generated by the slave
  cp_init.next_conn_params_update_delay   = NEXT_CONN_PARAMS_UPDATE_DELAY;  // this delay is used if the master denies the request and the slaves trys again
  cp_init.max_conn_params_update_count    = MAX_CONN_PARAMS_UPDATE_COUNT;   // the update is canceled after this number of update attemps; slave throws an error for this (see error handler)
  cp_init.start_on_notify_cccd_handle     = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail              = false;                          // if the update fails, the device won't disconneced because this parameter is set "false" here 

  cp_init.evt_handler = on_conn_params_evt;          // PPCP's event handler, see above
  cp_init.error_handler = conn_params_error_handler; // PPCP's error handler, see above

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

//! @brief Advertising event handler
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  //ret_code_t err_code;
  switch (ble_adv_evt)
  {
    case BLE_ADV_EVT_FAST:
      NRF_LOG_INFO("Fast advertising...");
      led_blinking(100, 50); // is advertising
    break;

    case BLE_ADV_EVT_IDLE:
      led_blinking(50, 200); // not advertising, led blinks slower
      //sleep_mode_enter()
    break;

    default:
    break;
  }
}

//! @brief Advertising initialization
static void advertising_init(void)
{
  ret_code_t err_code;
  ble_advertising_init_t init;
  memset(&init, 0, sizeof(init)); // clear memory of init

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;                       // add device name
  init.advdata.include_appearance = true;                               // device type, unknown here; false in example src
  init.advdata.flags  = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;    // general discovery mode

  //init.advdata.include_appearance = false;
  //init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
  
  // new
  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
  // /new

  init.config.ble_adv_fast_enabled = true;                              // fast advertisement
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;                 // because fast_enable=true, these two lines has to be _fast_ too!
  init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

  init.evt_handler = on_adv_evt; // event handler for advertising

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

  // if this is used, the start_advertising function won't be needed
  //uint32_t      err_code = NRF_SUCCESS;
  //ble_advdata_t advInitData;
  //ble_advdata_t srInitData;

  ///* Initialize structs. */
  //memset(&advInitData, 0, sizeof(ble_advdata_t));
  //memset(&srInitData, 0, sizeof(ble_advdata_t));
  
  ///* Prepare advertising data. */
  //advInitData.name_type = BLE_ADVDATA_FULL_NAME; // add device name
  //advInitData.include_appearance = true;// device type, unknown here; false in example src
  //advInitData.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

  ///* Prepare scan response data. */
  ////srInitData.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  ////srInitData.uuids_complete.p_uuids = m_adv_uuids;
  
  ///* Set advertising and scan response data. */
  //m_advertising.adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
  //m_advertising.adv_mode_current = BLE_ADV_MODE_IDLE;

  //m_advertising.adv_data.adv_data.p_data = m_advertising.enc_advdata;
  //m_advertising.adv_data.adv_data.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX;  
  
  //err_code = ble_advdata_encode(&advInitData, m_advertising.adv_data.adv_data.p_data, &m_advertising.adv_data.adv_data.len);
  //APP_ERROR_CHECK(err_code);
  ////BLEPE_LOG_PRINTF_WARNING("adv_data.len 0x%X",m_advertising.adv_data.adv_data.len);
  //NRF_LOG_WARNING("adv_data.len 0x%X", m_advertising.adv_data.adv_data.len);

  //m_advertising.adv_data.scan_rsp_data.p_data = m_advertising.enc_scan_rsp_data;
  //m_advertising.adv_data.scan_rsp_data.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX; 
  
  //err_code = ble_advdata_encode(&srInitData, m_advertising.adv_data.scan_rsp_data.p_data, &m_advertising.adv_data.scan_rsp_data.len);
  //APP_ERROR_CHECK(err_code);
  ////BLEPE_LOG_PRINTF_WARNING("scan_rsp_data.len 0x%X",m_advertising.adv_data.scan_rsp_data.len);   
  //NRF_LOG_WARNING("scan_rsp_data.len 0x%X",m_advertising.adv_data.scan_rsp_data.len);   

  ///*
  // *  Configure a initial advertising configuration. The advertising data and advertising parameters
  // *  can be changed later, but must be set to legal values here to define an advertising handle.
  // */
  //m_advertising.adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
  //m_advertising.adv_params.duration        = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
  //m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
  //m_advertising.adv_params.p_peer_addr     = NULL;
  //m_advertising.adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
  //m_advertising.adv_params.interval        = BLE_GAP_ADV_INTERVAL_MIN;

  //err_code = sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, &m_advertising.adv_data, &m_advertising.adv_params);
  //APP_ERROR_CHECK(err_code);

  //m_advertising.initialized = true;
}

//! @brief BLE event handler
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  ret_code_t err_code = NRF_SUCCESS;

  switch(p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Device is disconnected!");
      m_conn_handle = BLE_CONN_HANDLE_INVALID; // nec??
      nrf_gpio_pin_set(LED_2); // led off if connected; active-low
    break;

    case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("Device is connected!");
      nrf_gpio_pin_clear(LED_2); // led on if connected; active-low
      
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;               // get handle address of this connection
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle); // assign connection to the qwr that we know where to write
      APP_ERROR_CHECK(err_code);
    break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
      // If device wants to communicate faster, for example. Update automatically the bit rate then.
      NRF_LOG_DEBUG("PHY Update Request.");
      ble_gap_phys_t const phys = 
      {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
    break;

    // new:
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      // Pairing not supported
      // TODO maybe: enable pairing raspi
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
    break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
  }
}

//! @brief Activate sleep mode if nothing is processed
static void idle_state_handler(void)
{
  if(NRF_LOG_PROCESS() == false) // Has logger finished, are the buffers cleared?
  { 
    // power management starts, i.e., if nothing is processed, the management will activate the sleep mode
    nrf_pwr_mgmt_run();//TODO: maybe use acconeer's sleep mode?
  }
}

// @brief Start an advertisement
//not recommended in sdk15, see https://devzone.nordicsemi.com/f/nordic-q-a/37233/ble_advertising_init-returning-nrf_error_invalid_param-for-parameters-set-by-the-function-itself
static void advertising_start(void)
{
  ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST); // same time of advertisement here, used _FAST before as well!
  APP_ERROR_CHECK(err_code);
}



//! @brief Start the hardware parts which are necessary for BLE and the XM122
static void ble_xm122_inits(void)
{
  uart_init();                                // start the uart periphery

  nrf_drv_clock_lfclk_request(NULL);          // lfclk: low-frequency clock
  APP_ERROR_CHECK(nrf_drv_clock_init());      // initializing the nrf_drv_clock module 
  APP_ERROR_CHECK(nrf_drv_power_init(NULL));  // power module driver processes all the interrupts from power system
  
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));        // initializing the logs;                     both: log_init() see
  NRF_LOG_DEFAULT_BACKENDS_INIT();            // initializing default backends, for logging

  APP_ERROR_CHECK(app_timer_init());          // initializing the timer module
  APP_ERROR_CHECK(nrf_pwr_mgmt_init());       // initializing power management; sleep mode etc.
      
  nrf_delay_ms(10);                           // Delay in order to handle rampup of voltage from buck converter
  acc_hal_integration_xm122_init();           // start the RADAR sensor
  
  nrf_gpio_cfg_output(LED_2);                 // XB122's LED 

  // Init the BLE Stack:
  ret_code_t err_code;
  uint32_t ram_start = 0;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);
  
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);
  
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

  // Call the other BLE init functions in the right order!
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();
  NRF_LOG_INFO("BLE Base application started... ");
}

int main(void)
{
  /*
    Start the hardware parts which are necessary for BLE and the XM122
  */ 
  ble_xm122_inits();
  led_blinking(10, 200);   // Toggle the LED 10 times as a restart indicator; 50 ms off/on
  nrf_delay_ms(200);
  nrf_gpio_pin_set(LED_2); // led off; active-low

  advertising_start();    // start advertising

  /*
    Setup Envelope Service
  */
  printf("Acconeer software version %s\n", acc_version_get());//TODO: in extra function? hal just need here at the start, then no usage any more...
  acc_hal_t hal = *acc_hal_integration_get_implementation();
  acc_integration_set_periodic_wakeup(SUSPEND_TIME_BETWEEN_UPDATES_MS);
  hal.log.log_level = ACC_LOG_LEVEL_ERROR;

  if(!acc_rss_activate(&hal))
  {
    return EXIT_FAILURE;
  }

  acc_service_handle_t handle = create_envelope_service();//TODO: global due to interrupts?
  if(handle == NULL)
  {
    printf("acc_service_create() failed\n");
    return EXIT_FAILURE;
  }

  if(!acc_service_activate(handle))
  {
    printf("acc_service_activate() failed\n");
    acc_service_destroy(&handle);
    acc_rss_deactivate();
    return EXIT_FAILURE;
  }

  /*
    Data structures of the Envelope Service       
  */
  //TODO: global due to interrupts? start
  uint8_t byte_msg_metadata[BYTES_META_DATA_ALL];             // the metada as a message in bytes for uart
  acc_service_envelope_metadata_t metadata = { 0 };           // Start/length of sweep, signal length, stitch_count, Delta data point in m
  acc_service_envelope_get_metadata(handle, &metadata);
  //create_metadata_msg(byte_msg_metadata, &metadata);// TODO: replace with meta data radar + offset xyz
  //TODO: global due to interrupts? end

  uint16_t envelope_data[metadata.data_length];               // raw data of the RX-signal
  //uint16_t *envelope_data;                                  // raw data of the RX-signal, point to the memory which is allocated by the RSS
  acc_service_envelope_result_info_t result_info;             // missed_data, sensor_communication_error, data_saturated, data_quality_warning
  
  // 2* due to 1x16 bit |--> 2x8 bit; +2 due to two sync bytes; 
  // 3*4=12 due to 3 axis of the magnetic field sensor (x,y,z) and the sensor's data is 32 bit which needs 4 transmission bytes:
  const uint16_t msg_size = 2*metadata.data_length + 2 + 12*LENGTH_MGF_VECTOR; //TODO: global due to interrupts?
  // bytes of the uart message which contain the radar data & the magnetic field data:         
  uint8_t signal_msg[msg_size];                               

  
  while(true) 
  {
    idle_state_handler(); // sleep mode if nothing is processed
  } 
        
  // shut down the envelope service, by Acconeer
  acc_service_deactivate(handle);
  acc_service_destroy(&handle);
  acc_rss_deactivate();

  return EXIT_SUCCESS;
}