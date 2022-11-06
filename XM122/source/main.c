/**
 * @brief Sending the RADAR chip's data and the MMC's data via Bluetooth Low Energy (BLE) to the Raspberry Pi.
 * This device uses the peripheral role (p-role; server) and the Raspberry Pi the central role (c-role; client)
 *
 * @author Bennet Ladage
 * @date 2022-10-12
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 */

#include "pd_project_includes.h"  // includes for XM122, BLE, MMC and everything else...
#include "pd_project_defines.h"   // defines for XM122, RADAR, MMC, etc; BLE defines in this main
#include <complex.h>
#include "tfmini_plus.h"

/*
  Defines for the thresholds of the LiDAR's distance to determine whether a car parks or not
*/
#define LENGTH_PARKING_SPOT_CM  490 // LiDAR 1
#define WIDTH_PARKING_SPOT_CM   240 // LiDAR 0
#define THRESHOLD_DISTANCE_X_CM WIDTH_PARKING_SPOT_CM/2    // width; x as the mgf sensor, LiDAR0
#define THRESHOLD_DISTANCE_Y_CM LENGTH_PARKING_SPOT_CM/2   // length; y as the mgf sensor, LiDAR1
#define THRESHOLD_DISTANCE_Z_CM 100                        // height; z as the mgf sensor

/*
  Definitions for the application timer (RTC1)
*/
#define UPDATE_TIMESTAMP        APP_TIMER_TICKS(64000) // 65,000ms = 65s; update the timestamp every second: timestamp is second timer; timer_offset is ms timer
#define UPDATE_TIMESTAMP_SEC    64                     // time in seconds, must match the value above     

/*
  BLE Configuration; the Nordic UART Service (NUS) is used here
*/
#define APP_BLE_CONN_CFG_TAG    1
#define APP_BLE_OBSERVER_PRIO   3

// BLE's GAP parameters: PPCP (Periperal Preferred Connection Parameters)
#define DEVICE_NAME             "PD_Sensor"
#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN        /**< UUID type for the Nordic UART Service (vendor specific). */
#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(7.5, UNIT_1_25_MS)  // 7.5 time where peripheral has to answer central that he is still alive
#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(8, UNIT_1_25_MS)    // 8
#define SLAVE_LATENCY           0                                 // how often the peripheral (this device) skips the connection event/interval
#define CONN_SUP_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)   // time to wait until the link is lost after a connection event, i.e., this devices falls back to disconneced state

// BLE's advertising parameters:
#define APP_ADV_INTERVAL    64         // 40*0.625ms = 25ms     
#define APP_ADV_DURATION    0 //18000      // 5 ms time period for broadcasts

// BLE's PPCP (Peripheral Preferred Connection Parameters)
#define FIRST_CONN_PARMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)   
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3
#define BLE_TX_POWER 8  // dBm

#define DEAD_BEEF           0xDEADBEEF

// Defines for UART
#define UART_TX_BUFF_SIZE 128 // nec?
#define UART_RX_BUFF_SIZE 128

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

// Instance for I2C/TWI: Using TWI_1 here, because TWI_0 uses the same base address as SPI_0 which is used by the A111 chip
#define TWI_INSTANCE_ID 1
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID); 
static bool m_xfer_done = false; // A flag to indicate the I2C transfer state

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
uint16_t sleep_period = 0;  // TODO should be used for deep sleep mode
uint32_t timestamp = 0;     // stores unix timestamp in seconds

radar_config  cnfg_radar;
mmc_config    cnfg_mmc;
lidar_config  cnfg_lidar;

/*
  Application timer variables
*/
uint16_t time_offset_radar_ms = 0; // time_offset_radar_ms = counter_tor x cnfg_radar.sample_period_ms
uint16_t time_offset_mmc_ms = 0;   // time_offset_mmc_ms   = counter_tom x cnfg_mmc.sample_period_ms
uint16_t time_offset_lidar_ms = 0; // time_offset_lidar_ms = counter_tol x cnfg_lidar.sample_period_ms

uint32_t counter_tor = 0; // counter Time Offset Radar
uint32_t counter_tom = 0; // counter Time Offset Memsic
uint32_t counter_tol = 0; // counter Time Offset Lidar
uint32_t counter_tor_max = 0; // used for error handling; this counter is calculated by counter_max = floor(UPDATE_TIMESTAMP_SEC / maxmium sampling period in seconds) 
uint32_t counter_tom_max = 0;
uint32_t counter_tol_max = 0;

//bool timestamp_passed_first_update = false;
//uint16_t t_err_radar_ms = 0; // correct difference between updated timestamp and real one
//uint16_t t_err_mmc_ms = 0;
//uint16_t t_err_lidar_ms = 0;

/*
  Global variables for parking detection
*/
bool occupied = false; // Is parking space occupied? Y in lin. regression model

/*
  General measurement settings
*/
bool flag_setup = false;                    // is the setup of the sensors completed?
bool flag_connected = false;                // connected to central device?
bool flag_iq_setup_rx = false;              // flag indicates that the sensor configuration was received, i.e., the radar can be set up after this flag is true

/*
  Flags for radar, magnetic field sensor (mgf) and lidar measurements
*/
volatile bool flag_read_iq = false;     // triggered by timer, if set read radar and send ble messages
volatile bool flag_read_mmc = false;    // triggered by timer, if set read magnetic field sensor and send ble messages
volatile bool flag_read_lidar = false;  // triggered by timer, if set read lidar sensor and send ble messages
volatile bool flag_tx_complete = false; // triggered if a ble notification was transmitted successfully, is needed to check whether the tx buffer is empty

uint16_t nob_radar = BYTES_GENERAL_INFORMATION; // Number Of Bytes;  8+4N mag&phase

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
acc_service_handle_t iq_handle = NULL; // handle the RADAR chip's data

/*
  Global variables for the measurement results
*/
float complex * cmpl_radar_signal;

uint32_t mgf_x[NUMBER_MGF_SENSORS] = {0}; // [0]=sensor 0, [1]=sensor 1, ... 
uint32_t mgf_y[NUMBER_MGF_SENSORS] = {0};
uint32_t mgf_z[NUMBER_MGF_SENSORS] = {0};
float die_temperature[NUMBER_MGF_SENSORS] = {0};

uint16_t dist_lidar[NUMBER_LIDAR_SENSORS] = {0}; // [0]=sensor 0, [1]=sensor 1, [2]=sensor 2, ...
uint16_t strength_lidar[NUMBER_LIDAR_SENSORS] = {0};
uint16_t temperature_lidar[NUMBER_LIDAR_SENSORS] = {0};

/*
  I2C addresses of the LiDAR sensors
*/
uint8_t tfmp_addr[NUMBER_LIDAR_SENSORS] = {0};

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
  I2C / TWI functions by Bennet
*/
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    // if data transmission or receiving is finished
    if(p_event->type == NRF_DRV_TWI_EVT_DONE)
    {
      m_xfer_done = true; // reset the flag
    }
}

static ret_code_t twi_master_init(void)
{
  ret_code_t err_code;
              
  const nrf_drv_twi_config_t twi_config = {
    .scl                = TWI_CLK_Pin,               // SCL=23, XB122
    .sda                = TWI_DATA_Pin,              // SDA=21, XB122
    .frequency          = NRF_DRV_TWI_FREQ_100K,     // communication speed to 100k, (250k, 400k possible too)
    .interrupt_priority = APP_IRQ_PRIORITY_MID,      // interrupt priority, not high if soft-device is used
    .clear_bus_init     = true                       // needed to restore a hung up I2C device which is the LiDAR if it is connected to the bus
  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL); // initialize the twi
  APP_ERROR_CHECK(err_code);                                           // check if any error occured during initialization

  if(err_code == NRF_SUCCESS)
  {
    nrf_drv_twi_enable(&m_twi); // enable communication via TWI
    //while(nrf_drv_twi_is_busy(&m_twi)); // wait if TWI is busy
  }
  return err_code;
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

  acc_service_configuration_t iq_configuration = acc_service_iq_configuration_create();

  if(iq_configuration == NULL)
  {
    NRF_LOG_INFO("acc_service_iq_configuration_create() failed\n");
    acc_rss_deactivate();
    return false;
  }

  acc_update_iq(iq_configuration, start_m, length_m, downsampling, hwaas, profile, cutoff_lowpass);
  iq_handle = acc_service_create(iq_configuration);

  if(iq_handle == NULL)
  {
    NRF_LOG_INFO("acc_service_create() failed\n");
    acc_service_iq_configuration_destroy(&iq_configuration);
    acc_rss_deactivate();
    return false;
  }

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
  return true;
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
static void led_blinking(uint8_t iterations, uint16_t delay_ms)
{
  for(uint8_t idx=0; idx < iterations; idx++)
  {
    nrf_gpio_pin_toggle(LED_1);
    nrf_delay_ms(delay_ms);
  }
}

/*
  UART -- Read/Write LiDAR Sensor
*/
//void read_tfmini_via_uart(uint8_t lidar_number)
//{
//  uint8_t cmd = 65; // command which triggers the Arduino to send the LiDAR's data; 65='A'
//  uint8_t rx_buf = 0;
//  uint8_t rx_uart[7] = {0};

//  app_uart_put(cmd);
//  nrf_gpio_pin_toggle(LED_2);
//  NRF_LOG_INFO("---");

//  for(uint8_t k=0; k < 7; k++)
//  {
//    while(app_uart_get(&rx_buf) != NRF_SUCCESS); 
//    NRF_LOG_INFO("%d", rx_buf);
//    rx_uart[k] = rx_buf;
//  }      
//  dist_lidar[lidar_number] |= (rx_uart[1] << 8);
//  dist_lidar[lidar_number] |= rx_uart[2]; 
          
//  strength_lidar[lidar_number] |= (rx_uart[3] << 8);
//  strength_lidar[lidar_number] |= rx_uart[4];
          
//  temperature_lidar[lidar_number] |= (rx_uart[5] << 8);
//  temperature_lidar[lidar_number] |= rx_uart[6];
//}

/**@snippet [Handling the data received over UART] */
//void uart_event_handle(app_uart_evt_t * p_event)
//{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//    static uint8_t index = 0;
//    uint32_t       err_code;

//    switch (p_event->evt_type)
//    {
//        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;

//            NRF_LOG_DEBUG("[%d]: %x", index, data_array[index]);

//            //if ((data_array[index - 1] == '\n') ||
//            //    (data_array[index - 1] == '\r') ||
//            //    (index >= m_ble_nus_max_data_len))
//            //{
//            //    if (index > 1)
//            //    {
//            //        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
//            //        NRF_LOG_HEXDUMP_DEBUG(data_array, index);

//            //        do
//            //        {
//            //            uint16_t length = (uint16_t)index;
//            //            err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
//            //            if ((err_code != NRF_ERROR_INVALID_STATE) &&
//            //                (err_code != NRF_ERROR_RESOURCES) &&
//            //                (err_code != NRF_ERROR_NOT_FOUND))
//            //            {
//            //                APP_ERROR_CHECK(err_code);
//            //            }
//            //        } while (err_code == NRF_ERROR_RESOURCES);
//            //    }

//            //    index = 0;
//            //}
//            break;

//        case APP_UART_COMMUNICATION_ERROR:
//            //APP_ERROR_HANDLER(p_event->data.error_communication);
//            NRF_LOG_ERROR("Communication error occurred while handling UART: %08X", p_event->data.error_communication );
//            if( p_event->data.error_communication & UART_ERRORSRC_BREAK_Msk )
//            {
//                NRF_LOG_ERROR("   Break");
//            }
//            if( p_event->data.error_communication & UART_ERRORSRC_FRAMING_Msk )
//            {
//                NRF_LOG_ERROR("   Framing");
//            }
//            if( p_event->data.error_communication & UART_ERRORSRC_PARITY_Msk )
//            {
//                NRF_LOG_ERROR("   Parity");
//            }
//            if( p_event->data.error_communication & UART_ERRORSRC_OVERRUN_Msk )
//            {
//                NRF_LOG_ERROR("   Overrun");
//            }

//            break;

//        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;

//        default:
//            break;
//    }
//}

//static void uart_init(void)
//{
//    uint32_t                     err_code;
//    app_uart_comm_params_t const comm_params =
//    {
//        .rx_pin_no    = UART_RX,
//        .tx_pin_no    = UART_TX,
//        .rts_pin_no   = UART_RTS,
//        .cts_pin_no   = UART_CTS,
//        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
//        .use_parity   = false,
//#if defined (UART_PRESENT)
//        .baud_rate    = NRF_UART_BAUDRATE_115200
//#else
//        .baud_rate    = NRF_UARTE_BAUDRATE_115200
//#endif
//    };

//    APP_UART_FIFO_INIT(&comm_params,
//                       UART_RX_BUFF_SIZE,
//                       UART_TX_BUFF_SIZE,
//                       uart_event_handle,
//                       APP_IRQ_PRIORITY_LOWEST,
//                       err_code);
//    APP_ERROR_CHECK(err_code);
//}


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
    do
    {
      err_code = ble_nus_data_send(&m_nus, &nus_array[k*nus_msg_len], &nus_msg_len, m_conn_handle);
      NRF_LOG_INFO("%d err: %d", k, err_code);
    } while (err_code == NRF_ERROR_RESOURCES);
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
        if(p_evt->params.rx_data.p_data[0] == MSG_SEND_META_DATA)
        {
          uint8_t byte_msg[BYTES_MSG_SEND_META_DATA];
          uint16_t number_of_msgs = 0;
          
          NRF_LOG_INFO("--- metadata msg RX ---");

          // set how much bytes are needed for the transmission of a radar signal
          nob_radar += 4*iq_metadata.data_length;
          NRF_LOG_INFO(">>> NOB RADAR %d", nob_radar);
          
          // Get offset from all mgf sensors
          mmc_get_bridge_offset_only(&m_twi, &m_xfer_done, offset_xyz); // 0. sensor
          //TODO Sensor 1 - 5!!
          //for(uint8_t k=0; k < 18; k+=3)
          //{
          //  mmc_get_bridge_offset_only(offset_xyz+k); // increase start address of the array by 3 (axes)
          //}
          
          // Send BLE message which contains the metadata         
          create_msg_send_metadata(byte_msg, &iq_metadata, offset_xyz);      // build message
          calc_number_of_msgs(BYTES_MSG_SEND_META_DATA, &number_of_msgs);    // calc how much messages has to be send, should be 1
          ble_notify_array(byte_msg, number_of_msgs);                        // send array via BLE notification

          // Start the application timers
          err_code = app_timer_start(m_update_timestamp, UPDATE_TIMESTAMP, NULL);
          APP_ERROR_CHECK(err_code);
          NRF_LOG_INFO("cnfg_radar.sample_period_ms: %d", cnfg_radar.sample_period_ms);
          
          if(cnfg_radar.sample_period_ms != 0) // just start timer and sampling if sampling period is not 0
          {
            err_code = app_timer_start(m_sampling_period_radar, APP_TIMER_TICKS(cnfg_radar.sample_period_ms), NULL);
            APP_ERROR_CHECK(err_code);
          }
          if(cnfg_mmc.sample_period_ms != 0)
          {
            err_code = app_timer_start(m_sampling_period_mmc, APP_TIMER_TICKS(cnfg_mmc.sample_period_ms), NULL);
            APP_ERROR_CHECK(err_code);
          }
          if(cnfg_lidar.sample_period_ms != 0)
          {
            err_code = app_timer_start(m_sampling_period_lidar, APP_TIMER_TICKS(cnfg_lidar.sample_period_ms), NULL);
            APP_ERROR_CHECK(err_code);  
          }
        }

        // Setup all sensors
        else if(p_evt->params.rx_data.p_data[0] == MSG_SETUP_CONFIG)
        {
          NRF_LOG_INFO("--- MSG_SETUP_CONFIG RX ---");
          //uint16_t t_max_ms = 0;
          process_msg_setup_config(p_evt->params.rx_data.p_data, &cnfg_radar, &cnfg_mmc, &cnfg_lidar, &sleep_period, &timestamp);
          
          if(cnfg_radar.sample_period_ms != 0)
          {
            counter_tor_max = floor((1000*UPDATE_TIMESTAMP_SEC) / cnfg_radar.sample_period_ms); // *1000 --> convert in ms
            //t_max_ms = counter_tor_max * cnfg_radar.sample_period_ms;
            //if((1000*UPDATE_TIMESTAMP_SEC) % t_max_ms != 0)
            //{
            //  t_err_radar_ms = (1000*UPDATE_TIMESTAMP_SEC) - t_max_ms;
            //}
          }
          if(cnfg_mmc.sample_period_ms != 0)
          {
            counter_tom_max = floor((1000*UPDATE_TIMESTAMP_SEC) / cnfg_mmc.sample_period_ms);
            //t_max_ms = counter_tor_max*cnfg_mmc.sample_period_ms;
            //if((1000*UPDATE_TIMESTAMP_SEC) % t_max_ms != 0)
            //{
            //  t_err_mmc_ms = (1000*UPDATE_TIMESTAMP_SEC) - t_max_ms;
            //}
          }
          if(cnfg_lidar.sample_period_ms != 0)
          {
            counter_tol_max = floor((1000*UPDATE_TIMESTAMP_SEC) / cnfg_lidar.sample_period_ms);
            //t_max_ms = counter_tor_max*cnfg_lidar.sample_period_ms;
            //if((1000*UPDATE_TIMESTAMP_SEC) % t_max_ms != 0)
            //{
            //  t_err_lidar_ms = (1000*UPDATE_TIMESTAMP_SEC) - t_max_ms;
            //}
          }
          
          //time_reset_counter_ms = counter_max * sampling_period__ms_max; //debug only?

          NRF_LOG_INFO("x_r,max= %d (%d)ms, t_s=%d", counter_tor_max, (counter_tor_max*cnfg_radar.sample_period_ms), cnfg_radar.sample_period_ms);
          NRF_LOG_INFO("x_m,max= %d (%d)ms, t_s=%d", counter_tom_max, (counter_tor_max*cnfg_mmc.sample_period_ms), cnfg_mmc.sample_period_ms);
          NRF_LOG_INFO("x_l,max= %d (%d)ms, t_s=%d", counter_tol_max, (counter_tor_max*cnfg_lidar.sample_period_ms), cnfg_lidar.sample_period_ms);
          
          flag_iq_setup_rx = true; // sensor configuration received, i.e., the radar can be set up now

          NRF_LOG_INFO("Timestamp: %d", timestamp);
        }
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
      // maybe: enable pairing raspi
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
    break;
    
    // If tx event is complete, the tx buffer will be clear
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
  occupied = false;
  if(tfmp_get_data(&m_twi, &m_xfer_done, tfmp_addr, dist_lidar, strength_lidar, temperature_lidar) == TFMP_READY) // read LiDAR 0
  {
    if(tfmp_get_data(&m_twi, &m_xfer_done, tfmp_addr+1, dist_lidar+1, strength_lidar+1, temperature_lidar+1) == TFMP_READY) // read LiDAR 1
    {
      if((dist_lidar[1] < THRESHOLD_DISTANCE_Y_CM) && (dist_lidar[1] != 0) && (dist_lidar[0] < THRESHOLD_DISTANCE_X_CM) && (dist_lidar[0] != 0))
      //if(true)
      {
        occupied = true;
      } 
    }      
  }
  else
  {
    NRF_LOG_INFO("Errr det occ 0");
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
  //timestamp_passed_first_update = true;
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

  //if(timestamp_passed_first_update) // if timestamp was updated there could be a difference which is corrected
  //{
  //  time_offset_radar_ms = time_offset_radar_ms - t_err_radar_ms;
  //}

  //acc_data_iq(); // get_next_env doesn't work here?!?!? (both iq & low power env)
  flag_read_iq = true;
  
  //NRF_LOG_INFO("APPTIMER radar tstamp: %d s", timestamp);
  //NRF_LOG_INFO("APPTIMER radar offset: %d s", time_offset_radar_ms/1000);
}

//! @brief Send the mgf signal after its sampling period
static void apptimer_send_mmc_handler(void * p_context)
{
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

  //if(timestamp_passed_first_update) // if timestamp was updated there could be a difference which is corrected
  //{
  //  time_offset_mmc_ms = time_offset_mmc_ms - t_err_mmc_ms;
  //}
  flag_read_mmc = true;
  
  //NRF_LOG_INFO("APPTIMER mmc tstamp: %d s", timestamp);
  //NRF_LOG_INFO("APPTIMER mmc offset: %d s", time_offset_mmc_ms/1000);
}

//! @brief Send the lidar signal after its sampling period
static void apptimer_send_lidar_handler(void * p_context)
{
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

  //if(timestamp_passed_first_update) // if timestamp was updated there could be a difference which is corrected
  //{
  //  time_offset_lidar_ms = time_offset_lidar_ms - t_err_lidar_ms;
  //}
  flag_read_lidar = true;
  //NRF_LOG_INFO("APPTIMER lidar tstamp: %d s", timestamp);
  //NRF_LOG_INFO("APPTIMER lidar offset: %d s", time_offset_lidar_ms/1000);
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

static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    ble_cfg_t ble_cfg;
    memset(&ble_cfg, 0, sizeof ble_cfg);
    ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 50;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

//! @brief Start the hardware parts which are necessary for BLE and the XM122
static void ble_xm122_inits(void)
{
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
  ble_stack_init();

  // Call the other BLE init functions in the right order!
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();
  NRF_LOG_INFO("BLE Base application started... ");

  //mmc_init(&cnfg_mmc, &shadows);    // start the Memsic MMC5983MA magnetic field (mgf) sensor
  for(uint8_t k = 0; k < NUMBER_LIDAR_SENSORS; k++)
  {
    tfmp_addr[k] = k+0x10; // sensor0: addr 0x10, sensor1: addr 0x11, sensor2: addr 0x12, ...
  }
  //tfmp_init(tfmp_addr); // loop above later?
  //tfmp_init(tfmp_addr+1);
  
  twi_master_init(); // start the I2C/TWI instance

  acc_hal_integration_xm122_init(); // start the RADAR sensor
  timers_init();                    // start the application timers for sampling the radar, mfg and lidar; there is also a timer for updating the timestamp which will were received by a BLE message from the raspberry pi
}

static void ble_notify_radar_data(uint8_t * byte_msg, float complex * cmpl_radar_signal)
{
  uint16_t number_of_nus_msgs = 0; 
  uint16_t nus_msg_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE-3;   // data bytes per message, standard is 20 bytes, 3 byte overhead
  uint32_t err_code = 0;

  // Check LiDAR if parking spot is occupied: "supervised learning"
  determine_occupied_status();
 
  // Build BLE message
  create_msg_send_radar(byte_msg, &occupied, &timestamp, &time_offset_radar_ms, cmpl_radar_signal, iq_metadata.data_length);
  calc_number_of_msgs(nob_radar, &number_of_nus_msgs);  // how many ble messages need to be sent?     

  // send signal, i.e., number_of_nus_mgs x (ATT_MTU-3) byte messages in a row, but wait if tx buffer is full
  for(uint16_t k = 0; k < number_of_nus_msgs; k++) 
  { 
    while(!flag_tx_complete) // wait until txre buffer is empty     
    {
      //nrf_pwr_mgmt_run();
    }
    flag_tx_complete = false; // reset flag
    do
    {
      err_code = ble_nus_data_send(&m_nus, byte_msg+(k*nus_msg_len), &nus_msg_len, m_conn_handle);
    } while (err_code == NRF_ERROR_RESOURCES);
   //nrf_delay_us(2500); // BLE 5.0 frame takes between 44 and 2120 us; ~400us processing time python?
  } 
}

static void ble_notify_mgf_data(uint8_t * byte_msg)
{
  //uint8_t byte_msg[BYTES_MGF_TOTAL] = {0};
  uint16_t number_of_nus_msgs = 0;
  uint16_t nus_msg_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE-3;   // data bytes per message
  uint32_t err_code = 0;

  // Read mmc sensors
  for(uint8_t k=0; k < NUMBER_MGF_SENSORS; k++)
  {
    if(k==0) // TODO remove if all 6 sensors are connected!
    {
      mmc_get_magnetic_field_xyz(&m_twi, &m_xfer_done, mgf_x+k, mgf_y+k, mgf_z+k); // get current magnetic field
      die_temperature[k] = mmc_get_die_temperature(&m_twi, &m_xfer_done);        // read the mgf sensor's die temperature
    }
    //TODO set just one mgf sensor active due to their same I2C addresses... --> hardware workaround is needed for this... or new sensor
  }

  // Check LiDAR if parking spot is occupied: "supervised learning"
  determine_occupied_status();
 
  // Build BLE message
  create_msg_send_mmc(byte_msg, &occupied, &timestamp, &time_offset_mmc_ms, mgf_x, mgf_y, mgf_z, die_temperature);
  calc_number_of_msgs(BYTES_MGF_TOTAL, &number_of_nus_msgs);  // how many ble messages need to be sent?

  // send signal, i.e., number_of_nus_mgs x (ATT_MTU-3) byte messages in a row, but wait if tx buffer is full
  for(uint16_t k = 0; k < number_of_nus_msgs; k++) 
  { 
    while(!flag_tx_complete) // wait until tx buffer is empty     
    {
      //nrf_pwr_mgmt_run();
    }
    flag_tx_complete = false; // reset flag
    do
    {
      err_code = ble_nus_data_send(&m_nus, byte_msg+(k*nus_msg_len), &nus_msg_len, m_conn_handle);
    } while (err_code == NRF_ERROR_RESOURCES);
  } 
}

static void ble_notify_lidar_data(uint8_t * byte_msg)
{
  uint16_t number_of_nus_msgs = 0;
  uint16_t nus_msg_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE-3;   // data bytes per message
  uint32_t err_code = 0;

  // Read LiDAR sensors
  for(uint8_t k=0; k < NUMBER_LIDAR_SENSORS; k++)
  {
    if(tfmp_get_data(&m_twi, &m_xfer_done, tfmp_addr+k, dist_lidar+k, strength_lidar+k, temperature_lidar+k) != TFMP_READY)
    {
      dist_lidar[k] = 0;
      strength_lidar[k] = 0;
      temperature_lidar[k] = 0;
    }
  }
  
  // Check LiDAR if parking spot is occupied: "supervised learning"
  determine_occupied_status();
 
  // Build BLE message
  create_msg_send_lidar(byte_msg, &occupied, &timestamp, &time_offset_lidar_ms, dist_lidar, strength_lidar, temperature_lidar);
  calc_number_of_msgs(BYTES_LIDAR_TOTAL, &number_of_nus_msgs);  // how many ble messages need to be sent?

  // send signal, i.e., number_of_nus_mgs x (ATT_MTU-3) byte messages in a row, but wait if tx buffer is full
  for(uint16_t k = 0; k < number_of_nus_msgs; k++) 
  { 
    while(!flag_tx_complete) // wait until tx buffer is empty     
    {
      //nrf_pwr_mgmt_run();
    }
    flag_tx_complete = false; // reset flag
    do
    {
      err_code = ble_nus_data_send(&m_nus, byte_msg+(k*nus_msg_len), &nus_msg_len, m_conn_handle);
    } while (err_code == NRF_ERROR_RESOURCES);
  } 
}

int main(void)
{
  uint8_t * byte_msg = NULL; // pointer which allocates dynamically memory for a (row of) bluetooth message(s)
  
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
        NRF_LOG_INFO("--- acc setupt iq passed ---");
        flag_setup = true; // setup is done completely, TODO nec?
        break;
      }
      // Setup sensors, i.e, LiDARS, magnetometers
      mmc_init(&m_twi, &m_xfer_done, &cnfg_mmc, &shadows);    // start the Memsic MMC5983MA magnetic field (mgf) sensor
          
      // loop for all LiDARs
      for(uint8_t k=0; k < NUMBER_LIDAR_SENSORS; k++)
      {
        tfmp_init(&m_twi, &m_xfer_done, tfmp_addr+k, &(cnfg_lidar.internal_sampling_freq_Hz)); // setup  the k-th LiDAR
      }
    }
  }

  /*
    Main loop for measuring
  //*/
  float complex cmpl_radar_signal[iq_metadata.data_length]; // store the complex radar signal
  while(true)
  {
    if(flag_read_iq) // capture radar data
    {
      bool success = true;
      acc_service_iq_result_info_t result_info;
      uint32_t err_code = 0;
      
      flag_read_iq = false; // reset flag
      byte_msg = (uint8_t *)calloc(nob_radar, sizeof(uint8_t));//realloc(byte_msg, nob_radar*sizeof(uint8_t));; // [nob_radar];
      
      if(byte_msg != NULL) // NULL if no memory available
      {
        // Read RADAR sensor
        success = acc_service_iq_get_next(iq_handle, cmpl_radar_signal, iq_metadata.data_length, &result_info);
        if(!success)
        {
          NRF_LOG_INFO("acc_service_iq_get_next() failed\n");
        }
        else // send data only if reading was successful
        {
          ble_notify_radar_data(byte_msg, cmpl_radar_signal);
        }
      }
      free(byte_msg);      
    }

    if(flag_read_mmc) // capture mgf data
    {
      flag_read_mmc = false; // reset flag
      byte_msg = (uint8_t *)calloc(BYTES_MGF_TOTAL, sizeof(uint8_t));
      ble_notify_mgf_data(byte_msg);
      free(byte_msg); 
    }

    if(flag_read_lidar) // capture lidar data
    {
      flag_read_lidar = false; // reset flag
      byte_msg = (uint8_t *)calloc(BYTES_LIDAR_TOTAL, sizeof(uint8_t));
      ble_notify_lidar_data(byte_msg);
      free(byte_msg);
      NRF_LOG_INFO("D0: %d", dist_lidar[0]);
      NRF_LOG_INFO("S0: %d", strength_lidar[0]);
      NRF_LOG_INFO("v0: %d", temperature_lidar[0]);
      NRF_LOG_INFO("D1: %d", dist_lidar[1]);
      NRF_LOG_INFO("S1: %d", strength_lidar[1]);
      NRF_LOG_INFO("v1: %d", temperature_lidar[1]);       
      NRF_LOG_INFO("v1: %d", temperature_lidar[1]);       
    }
  }
  return EXIT_SUCCESS;
}