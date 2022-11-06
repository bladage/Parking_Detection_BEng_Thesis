/**
 * @brief Sending the RADAR chip's data and the MMC's data via Bluetooth Low Energy (BLE) to the Raspberry Pi.
 * This device uses the peripheral role (p-role; server) and the Raspberry Pi the central role (c-role; client)
 *
 * @author Bennet Ladage
 * @date 2022-09-29
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 */

#include "pd_project_includes.h"  // includes for XM122, BLE, MMC and everything else...
#include "pd_project_defines.h"   // defines for XM122, RADAR, MMC, etc; BLE defines in this main
#include <complex.h>
//#include "pd_ble_messages.h"

//BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT

/*
  Defines for the thresholds of the LiDAR's distance to determine whether a car parks or not
*/
#define THRESHOLD_DISTANCE_X_CM 150   // width; x as the mgf sensor
#define THRESHOLD_DISTANCE_Y_CM 200   // length; y as the mgf sensor
#define THRESHOLD_DISTANCE_Z_CM 100   // height; z as the mgf sensor

/*
  Definitions for the application timer (RTC1)
*/
#define UPDATE_TIMESTAMP        APP_TIMER_TICKS(65000) // 65,000ms = 65s; update the timestamp every second: timestamp is second timer; timer_offset is ms timer
#define UPDATE_TIMESTAMP_SEC    65                     // time in seconds, must match the value above     

/*
  BLE Configuration; the Nordic UART Service (NUS) is used here
*/
#define APP_BLE_CONN_CFG_TAG    1
#define APP_BLE_OBSERVER_PRIO   3

// BLE's GAP parameters: PPCP (Periperal Preferred Connection Parameters)
#define DEVICE_NAME             "PD_Sensor"
#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN        /**< UUID type for the Nordic UART Service (vendor specific). */
#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(7.5, UNIT_1_25_MS)  //,7.5 time where peripheral has to answer central that he is still alive
#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(8, UNIT_1_25_MS)  //,10
#define SLAVE_LATENCY           0                                 // how often the peripheral (this device) skips the connection event/interval
#define CONN_SUP_TIMEOUT        MSEC_TO_UNITS(2000, UNIT_10_MS)   // time to wait until the link is lost after a connection event, i.e., this devices falls back to disconneced state

// BLE's advertising parameters:
#define APP_ADV_INTERVAL    64         // 40*0.625ms = 25ms     
#define APP_ADV_DURATION    0 //18000      // 5 ms time period for broadcasts

// BLE's PPCP (Peripheral Preferred Connection Parameters)
#define FIRST_CONN_PARMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)   
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3
#define BLE_TX_POWER 8  // dBm

//#ifdef BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT 
//#undef BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT 
//#define BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT 14
//#endif 


#define DEAD_BEEF           0xDEADBEEF

// Instances for BLE
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);   // Nordic UART Service (NUS) instance
NRF_BLE_QWR_DEF(m_qwr);                             // queued writer instance; QWRS if connection with n>1 devices!
NRF_BLE_GATT_DEF(m_gatt);                           // define instance for GATT
BLE_ADVERTISING_DEF(m_advertising);                 // instance for advertising

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; // connection handle for BLE
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            // Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module.
static ble_uuid_t m_adv_uuids[]          =                                        // Universally unique service identifier.
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};
uint16_t tx_buffers_ble = 0;

/*
  Define instances for the application timer
*/
APP_TIMER_DEF(m_update_timestamp);
APP_TIMER_DEF(m_sampling_period_radar);
APP_TIMER_DEF(m_sampling_period_mmc);
APP_TIMER_DEF(m_sampling_period_lidar);

/*
  Sensor configuration parameters
*/
uint16_t sleep_period = 0;
uint32_t timestamp = 0;

radar_config  cnfg_radar;
mmc_config    cnfg_mmc;
lidar_config  cnfg_lidar;

/*
  Application timer variables
*/
uint16_t time_offset_radar_ms = 0; // time_offset_radar_ms = counter_tor x cnfg_radar.sample_period_ms
uint16_t time_offset_mmc_ms = 0;   // time_offset_mmc_ms   = counter_tom x cnfg_mmc.sample_period_ms
uint16_t time_offset_lidar_ms = 0; // time_offset_lidar_ms = counter_tol x cnfg_lidar.sample_period_ms

uint16_t counter_tor = 0; // counter Time Offset Radar
uint16_t counter_tom = 0; // counter Time Offset Memsic
uint16_t counter_tol = 0; // counter Time Offset Lidar
uint16_t counter_tor_max = 0; // used for error handling; this counter is calculated by counter_max = floor(UPDATE_TIMESTAMP_SEC / maxmium sampling period in seconds) 
uint16_t counter_tom_max = 0;
uint16_t counter_tol_max = 0;

/*
  Global variables for parking detection
*/
bool occupied = false; // Is parking space occupied? Y in lin. regression model

/*
  General measurement settings
*/
bool flag_setup = false;                    // setup of the sensors completed?
bool flag_connected = false;                // connected to central device?
uint16_t flag_handle_radar = 0;             // decide whether and which measurement is started
bool flag_iq_setup_rx = false;

/*
  Buffer and flags for RADAR measurements
*/
bool flag_read_iq = false;
bool flag_tx_complete = false;

uint16_t nob_radar = BYTES_GENERAL_INFORMATION; // Number Of Bytes; +8*metadata_radar.data_length, but not kown yet; 8: real part (32bit) & imaginary part (32bit) --> 8 = 2parts x 4bytes
uint8_t * byte_msg_radar; // buffer radar msg

/*
  Metadata RADAR & magnetic field sensor (mgf)
*/
acc_service_iq_metadata_t metadata_radar = {0};   // Start/length of sweep, signal length, stitch_count, Delta data point in m
acc_service_iq_metadata_t iq_metadata = { 0 };
uint32_t offset_xyz[18] = {0};                    // offset in raw samples of the mgf sensor; 3axes x 6sensors = 12
mmc_backup_registers shadows = {0,0,0,0};         // the MMC's shadow registers, i.e., backup of internal control registers 0 to 3

/*
  Global variables for the RADAR sensor
*/
//acc_service_handle_t radar_handle; // handle the RADAR chip's data
acc_service_handle_t iq_handle = NULL;

/*
  Global variables for the measurement results
*/
float complex * cmpl_radar_signal;

uint32_t mgf_x[NUMBER_MGF_SENSORS] = {0}; // [0]=sensor 0, [1]=sensor 1, ... [5]=sensor 5
uint32_t mgf_y[NUMBER_MGF_SENSORS] = {0};
uint32_t mgf_z[NUMBER_MGF_SENSORS] = {0};
float die_temperature[NUMBER_MGF_SENSORS] = {0};

uint16_t dist_lidar[NUMBER_LIDAR_SENSORS] = {0}; // [0]=sensor 0, [1]=sensor 1, [2]=sensor 2
uint16_t strength_lidar[NUMBER_LIDAR_SENSORS] = {0};
uint16_t temperature_lidar[NUMBER_LIDAR_SENSORS] = {0};


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
  Acconeer's functions: IQ service, modified by Bennet
*/
//! @brief Update IQ-Service
static void acc_update_iq(acc_service_configuration_t iq_configuration, float * start_m, float * length_m, uint16_t * downsampling, uint8_t * hwaas, uint8_t * profile, float * cutoff_lowpass)
{
  //acc_service_iq_output_format_set(iq_configuration, ACC_SERVICE_IQ_OUTPUT_FORMAT_INT16_COMPLEX);
  acc_service_iq_output_format_set(iq_configuration, ACC_SERVICE_IQ_OUTPUT_FORMAT_FLOAT_COMPLEX);

  acc_service_requested_start_set(iq_configuration, *start_m);
  acc_service_requested_length_set(iq_configuration, *length_m);

  acc_service_hw_accelerated_average_samples_set(iq_configuration, *hwaas);
  acc_service_profile_set(iq_configuration, *profile);
  acc_service_iq_downsampling_factor_set(iq_configuration, *downsampling);
  // TODO running avg ??
  //acc_service_power_save_mode_set(iq_configuration, ACC_POWER_SAVE_MODE_OFF); // later more interesting...
  
  if(*cutoff_lowpass <= 0.5) 
  {
    acc_service_iq_depth_lowpass_cutoff_ratio_set(iq_configuration, true, *cutoff_lowpass); // cutoff 0=smoothest, cutoff 0.5=filter off
    //acc_service_iq_depth_lowpass_cutoff_ratio_set(iq_configuration, false, *cutoff_lowpass); // choose cutoff frequency automatically
  }
}

static bool acc_setup_iq(float * start_m, float * length_m, uint16_t * downsampling, uint8_t * hwaas, uint8_t * profile, float * cutoff_lowpass)
{
  NRF_LOG_INFO("Acconeer software version %s\n", acc_version_get());
  const acc_hal_t *hal = acc_hal_integration_get_implementation();

  if(!acc_rss_activate(hal))
  {
    NRF_LOG_INFO("acc_rss_activate() failed\n");
    return false;
  }
  else{NRF_LOG_INFO("OK__1");}

  acc_service_configuration_t iq_configuration = acc_service_iq_configuration_create();

  if(iq_configuration == NULL)
  {
    NRF_LOG_INFO("acc_service_iq_configuration_create() failed\n");
    acc_rss_deactivate();
    return false;
  }
  else{NRF_LOG_INFO("OK__2");}

  acc_update_iq(iq_configuration, start_m, length_m, downsampling, hwaas, profile, cutoff_lowpass);
  NRF_LOG_INFO("OK__update");
  iq_handle = acc_service_create(iq_configuration);
  NRF_LOG_INFO("OK__create");

  if(iq_handle == NULL)
  {
    NRF_LOG_INFO("acc_service_create() failed\n");
    acc_service_iq_configuration_destroy(&iq_configuration);
    acc_rss_deactivate();
    return false;
  }
  else{NRF_LOG_INFO("OK__3");}

  acc_service_iq_configuration_destroy(&iq_configuration);
  acc_service_iq_get_metadata(iq_handle, &iq_metadata);

  NRF_LOG_INFO("Start: %d mm\n", (int)(iq_metadata.start_m * 1000.0f));
  NRF_LOG_INFO("Length: %u mm\n", (unsigned int)(iq_metadata.length_m * 1000.0f));
  NRF_LOG_INFO("Data length: %u\n", (unsigned int)(iq_metadata.data_length));

  if(!acc_service_activate(iq_handle))
  {
    NRF_LOG_INFO("acc_service_activate() failed\n");
    acc_service_destroy(&iq_handle);
    acc_rss_deactivate();
    return false;
  }
  else{NRF_LOG_INFO("OK__4");}
  return true;
}

static void acc_data_iq(void)
{
  bool                         success    = true;
  float complex                data[iq_metadata.data_length];
  //acc_int16_complex_t * data;
  acc_service_iq_result_info_t result_info;

  //success = acc_service_iq_get_next_by_reference(iq_handle, &data, &result_info);
  success = acc_service_iq_get_next(iq_handle, data, iq_metadata.data_length, &result_info);
  if(!success)
  {
    NRF_LOG_INFO("acc_service_iq_get_next() failed\n");
  }
  else
  {
    NRF_LOG_INFO("SUCC!!!");
  }
    
    //for(uint16_t i = 0; i < iq_metadata.data_length; i++)
    //{
    //  NRF_LOG_INFO("%d(%d, %d)\t", i, (int)(cabsf(data[i]) * 1000), (int)(cargf(data[i]) * 1000));
    //}
}

static int acc_shutdown_iq(void)
{
  bool deactivated = acc_service_deactivate(iq_handle);
  acc_service_destroy(&iq_handle);
  acc_rss_deactivate();

  if(deactivated) //&& success)
  {
    NRF_LOG_INFO("Application finished OK\n");
    return EXIT_SUCCESS;
  }
  return EXIT_FAILURE;
}


/*
  Misc
*/
//! @brief Toggle the LED
static void led_blinking(uint8_t iterations, uint8_t delay_ms)
{
  for(uint8_t idx=0; idx < iterations; idx++)
  {
    nrf_gpio_pin_toggle(LED_1);
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

//! @brief Error handler for queue writer */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

//@brief This functions return the how much nus messages/frames have to be send for the whole data, i.e., radar signal and all 3 axes of the mgf sensor
static void calc_number_of_msgs(const uint16_t lenght_in_bytes, uint16_t * number_of_nus_msgs)
{
  uint16_t nus_msg_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE-3;   // data bytes per message
  //uint16_t number_of_nus_msgs = 0;                          // How many nus messages have to be send that the whole data array is transmitted?

  if(lenght_in_bytes % nus_msg_len == 0)
  {
    *number_of_nus_msgs = lenght_in_bytes / nus_msg_len;
  }
  else
  {              
    *number_of_nus_msgs = (lenght_in_bytes / nus_msg_len) + 1;       // round: ceil(), int division           
  }
  //*bytes_msgs = nus_msg_len * (*number_of_nus_msgs); // How much data bytes are needed to transmit all frames?
}

//@brief Split the nus_array into package sizes which are transmittable via BLE
static void ble_notify_array(uint8_t * nus_array, const uint16_t number_of_nus_msgs)
{
  uint16_t nus_msg_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE-3;   // data bytes per message, standard is 20 bytes, 3 byte overhead
  uint32_t err_code = 0;
  NRF_LOG_INFO("I have to send %d msgs!", number_of_nus_msgs);
  // send signal, i.e., number_of_nus_mgs x (ATT_MTU-3) byte messages in a row
  for(uint16_t k = 0; k < number_of_nus_msgs; k++) 
  { 
    //while(!flag_tx_buffer_ready); // wait until flag is set

    do
    {
      err_code = ble_nus_data_send(&m_nus, &nus_array[k*nus_msg_len], &nus_msg_len, m_conn_handle);
      NRF_LOG_INFO("%d err: %d", k, err_code);
    } while (err_code == NRF_ERROR_RESOURCES);
    //flag_tx_buffer_ready = false;
    //while(true)
    //{
    //  if(err_code == BLE_ERROR_NO_TX_BUFFERS || err_code == NRF_ERROR_INVALID_STATE || err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    //  {       
    //    break;
    //  }
    //  else if (err_code != NRF_SUCCESS) 
    //  {
    //    APP_ERROR_HANDLER(err_code);
    //  }
    //}
    NRF_LOG_INFO("Msg no. %d was send!", k);
  }  
}

//! @brief Handle NUS events, i.e., receiving data
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    /*
      Respond to the message of the RX data
    */
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
        NRF_LOG_INFO("P_DATA: %d", p_evt->params.rx_data.p_data[0]);

        // Send the metdata of all sensors
        if(p_evt->params.rx_data.p_data[0] == MSG_SEND_META_DATA && flag_setup)
        {
          uint8_t byte_msg[BYTES_MSG_SEND_META_DATA];
          uint16_t number_of_msgs = 0;
          
          // set how much bytes are needed for the transmission of a radar signal
          nob_radar += 8*metadata_radar.data_length;

          // Testdata
          //metadata_radar.data_length = 50;
          //metadata_radar.depth_lowpass_cutoff_ratio = 0.12f;
          //metadata_radar.length_m = 0.50f;
          //metadata_radar.start_m = 0.08f;
          //metadata_radar.step_length_m = 0.000432;
          //metadata_radar.stitch_count = 0xEF98;
          
          // Get offset from all mgf sensors
          mmc_get_bridge_offset_only(offset_xyz); // 0. sensor
          //TODO Sensor 1 - 5!!
          //for(uint8_t k=0; k < 18; k+=3)
          //{
          //  mmc_get_bridge_offset_only(offset_xyz+k); // increase start address of the array by 3 (axes)
          //}
          
          // Send BLE message which contains the metadata         
          create_msg_send_metadata(byte_msg, &iq_metadata, offset_xyz);      // build message
          calc_number_of_msgs(BYTES_MSG_SEND_META_DATA, &number_of_msgs);    // calc how much messages has to be send
          ble_notify_array(byte_msg, number_of_msgs);                        // send array via BLE notification

          // Start the application timers
          err_code = app_timer_start(m_update_timestamp, UPDATE_TIMESTAMP, NULL);
          APP_ERROR_CHECK(err_code);
          err_code = app_timer_start(m_sampling_period_radar, APP_TIMER_TICKS(cnfg_radar.sample_period_ms), NULL);
          APP_ERROR_CHECK(err_code);
          err_code = app_timer_start(m_sampling_period_mmc, APP_TIMER_TICKS(cnfg_mmc.sample_period_ms), NULL);
          APP_ERROR_CHECK(err_code);
          err_code = app_timer_start(m_sampling_period_lidar, APP_TIMER_TICKS(cnfg_lidar.sample_period_ms), NULL);
          APP_ERROR_CHECK(err_code);  
        }

        // Setup all sensors
        else if(p_evt->params.rx_data.p_data[0] == MSG_SETUP_CONFIG)
        {
          process_msg_setup_config(p_evt->params.rx_data.p_data, &cnfg_radar, &cnfg_mmc, &cnfg_lidar, &sleep_period, &timestamp);
          
          counter_tor_max = floor((1000*UPDATE_TIMESTAMP_SEC) / cnfg_radar.sample_period_ms); // *1000 --> convert in ms
          counter_tom_max = floor((1000*UPDATE_TIMESTAMP_SEC) / cnfg_mmc.sample_period_ms);
          counter_tol_max = floor((1000*UPDATE_TIMESTAMP_SEC) / cnfg_lidar.sample_period_ms);
          //time_reset_counter_ms = counter_max * sampling_period__ms_max; //debug only?

          NRF_LOG_INFO("x_r,max= %d (%d)ms, t_s=%d", counter_tor_max, (counter_tor_max*cnfg_radar.sample_period_ms), cnfg_radar.sample_period_ms);
          NRF_LOG_INFO("x_m,max= %d (%d)ms, t_s=%d", counter_tom_max, (counter_tor_max*cnfg_mmc.sample_period_ms), cnfg_mmc.sample_period_ms);
          NRF_LOG_INFO("x_l,max= %d (%d)ms, t_s=%d", counter_tol_max, (counter_tor_max*cnfg_lidar.sample_period_ms), cnfg_lidar.sample_period_ms);
          
          // Setup radar sensor (IQ service)
          //acc_setup_iq(&(cnfg_radar.start_m), &(cnfg_radar.length_m), &(cnfg_radar.downsampling_factor), &(cnfg_radar.hwaas), &(cnfg_radar.profile),  &(cnfg_radar.cutoff_lowpass));
          flag_iq_setup_rx = true;
          // TODO setup sensors (lidar)

          //flag_setup = true;
          NRF_LOG_INFO("Timestamp: %d", timestamp);
        }
          
        // Case if invalid command was received
        else
        {
          uint8_t data_array_err[BLE_NUS_MAX_DATA_LEN] = {'I', 'n', 'v', 'a', 'l', 'i', 'd', ' ', 'M', 'S', 'G', '!', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
          uint16_t nus_msg_len = BLE_NUS_MAX_DATA_LEN;    // maximum message size, should be 244 byte

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
  
  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;                              // fast advertisement
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;                 // because fast_enable=true, these two lines has to be _fast_ too!
  init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

  //init.config.ble_adv_primary_phy      = BLE_GAP_PHY_2MBPS; // 2 Mbit/s mode enabled
  //init.config.ble_adv_secondary_phy    = BLE_GAP_PHY_2MBPS; 
  //init.config.ble_adv_extended_enabled = true;  // data length extension?

  init.evt_handler = on_adv_evt; // event handler for advertising

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

  err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, BLE_TX_POWER); // set TX power for adv
  APP_ERROR_CHECK(err_code);
  if(err_code == NRF_SUCCESS)
  {
    NRF_LOG_INFO("Changed successfully the TX power (ADV_ROLE)!");
  }
}

//! @brief BLE event handler
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  // Enable AUTO or 2 MBPS mode 
  ret_code_t err_code = NRF_SUCCESS;
  ble_gap_phys_t const phys = 
  {
    .rx_phys = BLE_GAP_PHY_AUTO, //BLE_GAP_PHY_2MBPS, 
    .tx_phys = BLE_GAP_PHY_AUTO, //BLE_GAP_PHY_2MBPS,  
  };

  switch(p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Device is disconnected!");
      flag_connected = false;
      m_conn_handle = BLE_CONN_HANDLE_INVALID; // nec??
      nrf_gpio_pin_set(LED_1); // led off if connected; active-low
    break;

    case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("Device is connected!");
      flag_connected = true;
      nrf_gpio_pin_clear(LED_1); // led on if connected; active-low
      
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;               // get handle address of this connection
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle); // assign connection to the qwr that we know where to write
      APP_ERROR_CHECK(err_code);

      err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, BLE_TX_POWER); // set TX power for adv/conn, see https://devzone.nordicsemi.com/f/nordic-q-a/37928/just-set-tx-power
      APP_ERROR_CHECK(err_code);
      if(err_code == NRF_SUCCESS)
      {
        NRF_LOG_INFO("Changed successfully the TX power (CONN ROLE)!");
      }
      
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      if(err_code == NRF_SUCCESS)
      {
        if(phys.rx_phys == BLE_GAP_PHY_2MBPS && phys.tx_phys == BLE_GAP_PHY_2MBPS)
        {
          NRF_LOG_INFO("Changed successfully the bit rate to 2 MBPS! (CONNECTED)");
        }
        else
        {
          NRF_LOG_INFO("Changed successfully the bit rate (auto mode)! (CONNECTED)");
        }
        
        // Get the numbers of ble messages that can be buffered
        //tx_buffers_ble = sd_ble_tx_packet_count_get()
      }
    break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
      // If device wants to communicate faster, for example. Update automatically the bit rate then.
      NRF_LOG_DEBUG("PHY Update Request.");
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
      if(err_code == NRF_SUCCESS)
      {
        if(phys.rx_phys == BLE_GAP_PHY_2MBPS && phys.tx_phys == BLE_GAP_PHY_2MBPS)
        {
          NRF_LOG_INFO("Changed successfully the bit rate to 2 MBPS! (PHY_UPDATE)");
        }
        else
        {
          NRF_LOG_INFO("Changed successfully the bit rate (auto mode)! (PHY_UPDATE)");
        }
      }
    break;

    // new:
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      // Pairing not supported
      // TODO maybe: enable pairing raspi
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
    break;
    
    // TODO AGAINST tx buffer problem???? DOES IT WORK?
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      flag_tx_complete = true;
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

/*
  Function to setup the application timer and functions which are used inside the app timer events
*/
//! @brief Use LiDAR to determine whether the cars parks or not
static void determine_occupied_status(void)
{
  uint16_t distance_y_cm = 0; // length
  uint16_t distance_x_cm = 0; // width
  uint16_t distance_z_cm = 0; // height(?)

  //TODO read distance LiDAR

  if((distance_y_cm <= THRESHOLD_DISTANCE_Y_CM) || (distance_x_cm <= THRESHOLD_DISTANCE_X_CM)) // && distances != 0 (?). Does d > d_max lead to d = 0 ?
  {
    occupied = true;
  }
  else
  {
    occupied = false;
  }
}

//! @brief Update timestamp after UPDATE_TIMESTAMP_SEC seconds and reset the counters of the periods to prevent an overflow of the 16 bit variable
static void apptimer_update_timestamp_handler(void * p_context)
{ 
  UNUSED_PARAMETER(p_context); 
  
  timestamp += UPDATE_TIMESTAMP_SEC; // update timestamp, i.e., if the timer ran 1s, then add 1s to the unix timestamp (seconds since 1970-01-01) 
  counter_tor = 0; // the counters has to be reset if the timestamp is updated. Otherwise, the refernce is wrong
  counter_tom = 0;
  counter_tol = 0;
  NRF_LOG_INFO("APPTIMER timestamp: %d s", timestamp);
  NRF_LOG_INFO("----------");
}

//! @brief Send the radar signal after its sampling period
static void apptimer_send_radar_handler(void * p_context)
{
  uint32_t timestamp_temporary = 0;
  uint16_t counter_temporary = 0;
  UNUSED_PARAMETER(p_context);  
  
  counter_tor++;

  if(counter_tor > counter_tor_max) // i.e., timestamp isn't updated yet because this interrupt handler was faster. Then, the counters arent cleared and overflow may occurs!
  {
     timestamp_temporary = timestamp + UPDATE_TIMESTAMP_SEC;
     counter_temporary = counter_tor - counter_tor_max;
     time_offset_radar_ms = counter_temporary * cnfg_radar.sample_period_ms;
  }
  else
  {
     time_offset_radar_ms = counter_tor * cnfg_radar.sample_period_ms;
  }

  //acc_data_iq(); // get_next_env doesn't work here?!?!? (both iq & low power env)
  flag_read_iq = true;
}

static void send_iq_data(void)
{
  uint16_t bytes_radar = BYTES_GENERAL_INFORMATION+8*metadata_radar.data_length; // 8: real part (32bit) & imaginary part (32bit) --> 8 = 2parts x 4bytes
  uint8_t byte_msg[bytes_radar];
  uint16_t number_of_nus_msgs = 0; 
  bool success = true;
  float complex cmpl_radar_signal[iq_metadata.data_length];
  //acc_int16_complex_t * data;
  acc_service_iq_result_info_t result_info;

  //success = acc_service_iq_get_next_by_reference(iq_handle, &data, &result_info);
  success = acc_service_iq_get_next(iq_handle, cmpl_radar_signal, iq_metadata.data_length, &result_info);
  if(!success)
  {
    NRF_LOG_INFO("acc_service_iq_get_next() failed\n");
  }
  else
  {
    NRF_LOG_INFO("SUCC!!!");
  }

  //TODO Read radar sensor, remove test data
  //float complex cmpl_test[metadata_radar.data_length];
  //for(int16_t k=0; k < metadata_radar.data_length; k++)
  //{
  //  cmpl_test[k] = k-(metadata_radar.data_length/2);
  //  //NRF_LOG_INFO("re[%d]: %d", k, crealf(cmpl_test[k]));
  //  //NRF_LOG_INFO("im[%d]: %d", k, cimagf(cmpl_test[k]));
  //  //NRF_LOG_INFO("mag[%d]: %d", k, cabsf(cmpl_test[k])*100);
  //  //NRF_LOG_INFO("phi[%d]: %d", k, cargf(cmpl_test[k])*100);
  //}
  ////NRF_LOG_INFO("-----");
  ////cmpl_test[0] = -1;
  ////cmpl_test[1] = 1;
  ////cmpl_test[2] = -500;
  ////cmpl_test[3] = 6500;
  //cmpl_radar_signal = cmpl_test;
  
  // Check LiDAR if parking spot is occupied: "supervised learning"
  determine_occupied_status();
 
  // Build BLE message
  create_msg_send_radar(byte_msg, &occupied, &timestamp, &time_offset_radar_ms, cmpl_radar_signal, metadata_radar.data_length);
  //for(uint16_t k=0; k < bytes_radar; k++)
  //{
  //  NRF_LOG_INFO("msg[%d]: %d", k, byte_msg[k]);
  //}

  // Send BLE message
  calc_number_of_msgs(bytes_radar, &number_of_nus_msgs);  // how many ble messages need to be sent?
  if(flag_connected)
  {
    ble_notify_array(byte_msg, number_of_nus_msgs); // send ble message(s) if connection is there
  }
  else // TODO remove after debugging
  {
    NRF_LOG_INFO("I won't send anything. I'm disconnected!!");
  }
  NRF_LOG_INFO("APPTIMER radar: %d ms", time_offset_radar_ms);  
}

//! @brief Send the mgf signal after its sampling period
static void apptimer_send_mmc_handler(void * p_context)
{
  uint8_t byte_msg[BYTES_MGF_TOTAL] = {0};
  uint16_t number_of_nus_msgs = 0;
  uint32_t timestamp_temporary = 0;
  uint16_t counter_temporary = 0;
  UNUSED_PARAMETER(p_context);  

  counter_tom++;

  if(counter_tom > counter_tom_max) // i.e., timestamp isn't updated yet because this interrupt handler was faster. Then, the counters arent cleared and overflow may occurs!
  {
     timestamp_temporary = timestamp + UPDATE_TIMESTAMP_SEC;
     counter_temporary = counter_tom - counter_tom_max;
     time_offset_mmc_ms = counter_temporary * cnfg_mmc.sample_period_ms;
  }
  else
  {
     time_offset_mmc_ms = counter_tom * cnfg_mmc.sample_period_ms;
  }

  // Read mmc sensors
  for(uint8_t k=0; k < NUMBER_MGF_SENSORS; k++)
  {
    if(k==0) // TODO remove if all 6 sensors are connected!
    {
      mmc_get_magnetic_field_xyz(mgf_x+k, mgf_y+k, mgf_z+k); // get current magnetic field
      die_temperature[k] = mmc_get_die_temperature();        // read the mgf sensor's die temperature
      //NRF_LOG_INFO("%d.(x|y|z)=(%d|%d|%d)", k, mgf_x[k], mgf_y[k], mgf_z[k]);
      //NRF_LOG_INFO("%d.vartheta=%d C", k, die_temperature[k]);
    }
    //TODO set just one mgf sensor active due to their same I2C addresses... --> hardware workaround is needed for this... or new sensor
  }

  // Check LiDAR if parking spot is occupied: "supervised learning"
  determine_occupied_status();
 
  // Build BLE message
  create_msg_send_mmc(byte_msg, &occupied, &timestamp, &time_offset_mmc_ms, mgf_x, mgf_y, mgf_z, die_temperature);

  // Send BLE message
  calc_number_of_msgs(BYTES_MGF_TOTAL, &number_of_nus_msgs);  // how many ble messages need to be sent?
  if(flag_connected)
  {
    ble_notify_array(byte_msg, number_of_nus_msgs); // send ble message(s) if connection is there
  }
  else // TODO remove after debugging
  {
    NRF_LOG_INFO("I won't send anything. I'm disconnected!!");
  }
  NRF_LOG_INFO("APPTIMER mmc: %d ms", time_offset_mmc_ms);
}

//! @brief Send the lidar signal after its sampling period
static void apptimer_send_lidar_handler(void * p_context)
{
  uint8_t byte_msg[BYTES_LIDAR_TOTAL] = {0};
  uint16_t number_of_nus_msgs = 0;
  uint32_t timestamp_temporary = 0;
  uint16_t counter_temporary = 0;
  UNUSED_PARAMETER(p_context);  

  counter_tol++;

  if(counter_tol > counter_tol_max) // i.e., timestamp isn't updated yet because this interrupt handler was faster. Then, the counters arent cleared and overflow may occurs!
  {
     timestamp_temporary = timestamp + UPDATE_TIMESTAMP_SEC;
     counter_temporary = counter_tol - counter_tol_max;
     time_offset_lidar_ms = counter_temporary * cnfg_lidar.sample_period_ms;
  }
  else
  {
     time_offset_lidar_ms = counter_tol * cnfg_lidar.sample_period_ms;
  }

  // TODO Read real LiDAR sensors, remove test data here
  for(uint16_t k=0; k < NUMBER_LIDAR_SENSORS; k++)
  {
    dist_lidar[k] = k; // 0,1,2
    strength_lidar[k] = k*10; // 0,10,20
    temperature_lidar[k] = k*1000; // 0,1000,2000
  }

  // Check LiDAR if parking spot is occupied: "supervised learning"
  determine_occupied_status();
 
  // Build BLE message
  create_msg_send_lidar(byte_msg, &occupied, &timestamp, &time_offset_lidar_ms, dist_lidar, strength_lidar, temperature_lidar);

  // Send BLE message
  calc_number_of_msgs(BYTES_LIDAR_TOTAL, &number_of_nus_msgs);  // how many ble messages need to be sent?
  if(flag_connected)
  {
    ble_notify_array(byte_msg, number_of_nus_msgs); // send ble message(s) if connection is there
  }
  else // TODO remove after debugging
  {
    NRF_LOG_INFO("I won't send anything. I'm disconnected!!");
  }
  NRF_LOG_INFO("APPTIMER lidar: %d ms", time_offset_lidar_ms);
}

//! @brief Setup of every timer module, assign a instance, repeated or single shot mode, and their interrupt handler
static void timers_init(void)
{
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&m_update_timestamp, APP_TIMER_MODE_REPEATED, apptimer_update_timestamp_handler);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&m_sampling_period_radar, APP_TIMER_MODE_REPEATED, apptimer_send_radar_handler);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&m_sampling_period_mmc, APP_TIMER_MODE_REPEATED, apptimer_send_mmc_handler);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&m_sampling_period_lidar, APP_TIMER_MODE_REPEATED, apptimer_send_lidar_handler);
  APP_ERROR_CHECK(err_code);
}

//! @brief Start the hardware parts which are necessary for BLE and the XM122
static void ble_xm122_inits(void)
{
  //uart_init();                                // start the uart periphery

  nrf_drv_clock_lfclk_request(NULL);          // lfclk: low-frequency clock
  APP_ERROR_CHECK(nrf_drv_clock_init());      // initializing the nrf_drv_clock module 
  APP_ERROR_CHECK(nrf_drv_power_init(NULL));  // power module driver processes all the interrupts from power system
  
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));        // initializing the logs;                     both: log_init() see
  NRF_LOG_DEFAULT_BACKENDS_INIT();            // initializing default backends, for logging

  APP_ERROR_CHECK(app_timer_init());          // initializing the timer module
  APP_ERROR_CHECK(nrf_pwr_mgmt_init());       // initializing power management; sleep mode etc.
      
  nrf_delay_ms(10);                           // Delay in order to handle rampup of voltage from buck converter
  nrf_gpio_cfg_output(LED_1);                 // XB122's LED 

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

  mmc_init(&cnfg_mmc, &shadows);    // start the Memsic MMC5983MA magnetic field (mgf) sensor
  acc_hal_integration_xm122_init(); // start the RADAR sensor
  timers_init();                    // start the application timers for sampling the radar, mfg and lidar; there is also a timer for updating the timestamp which will were received by a BLE message from the raspberry pi
  //acc_setup_iq(); // start iq service
}

int main(void)
{
  /*
    Start the hardware parts which are necessary for BLE, the XM122 (RADAR) and the Memsic Sensor (MMC)
  */ 
  NRF_LOG_INFO("Debugging started!!");
  ble_xm122_inits();       // activate BLE, RADAR, MMC
  led_blinking(10, 200);   // Toggle the LED 10 times as a restart indicator; 200 ms off/on
  nrf_delay_ms(200);
  nrf_gpio_pin_set(LED_1); // led off; active-low

  advertising_start();    // start advertising

  /*
    Get the data to configure the sensors' parameters and do the setup
  */
  while(true)
  {
    if(flag_iq_setup_rx)
    {
      if(acc_setup_iq(&(cnfg_radar.start_m), &(cnfg_radar.length_m), &(cnfg_radar.downsampling_factor), &(cnfg_radar.hwaas), &(cnfg_radar.profile),  &(cnfg_radar.cutoff_lowpass)))
      {
        flag_setup = true; // setup is done completly
        break;
      }
    }
  }

  /*
    Main loop for measuring
  */
  while(true)
  {
    if(flag_read_iq)
    {
      // read radar, fill buffer
      // wait until tx buffer is empty
       while(flag_tx_complete == false)
       {
          break;
       }
       flag_tx_complete = false; // reset flag

      // send one message
      send_iq_data();
      flag_read_iq = false;

    }
    else
    {
      idle_state_handler();
    }
  }
  
  return EXIT_SUCCESS;
}