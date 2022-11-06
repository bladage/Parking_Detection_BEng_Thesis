/*
  MAYBE: not needed, all in main.c????
*/

// Defines for low power envelope service
#ifndef SUSPEND_TIME_BETWEEN_UPDATES_MS
#define SUSPEND_TIME_BETWEEN_UPDATES_MS (10000) // 0.1Hz=10000
#endif





//#ifndef POWER_SAVE_MODE
//#define POWER_SAVE_MODE OFF
//#endif

//#ifndef RANGE_LENGTH
//#define RANGE_LENGTH (0.51f) // due to downsampling 0.5f less then 0.5, ~0.49 hence 0.51f
//#endif

//#ifndef RANGE_START
//#define RANGE_START (0.081f)  // due to downsampling 0.8f less then 0.8, ~0.79 hence 0.81f
//#endif

//#ifndef SERVICE_PROFILE
//#define SERVICE_PROFILE 1
//#endif

//#define PASTER(x, y)    x ## y
//#define EVALUATOR(x, y) PASTER(x, y)

//#define SELECTED_POWER_SAVE_MODE (EVALUATOR(ACC_POWER_SAVE_MODE_, POWER_SAVE_MODE))
//#define SELECTED_SERVICE_PROILE  (EVALUATOR(ACC_SERVICE_PROFILE_, SERVICE_PROFILE))

//#ifndef HWAAS
//#define HWAAS (10)   // default 10
//#endif

//#ifndef DOWNSAMPLING_FACTOR
//#define DOWNSAMPLING_FACTOR (4)
//#endif

// Defines for the 8 bit frame
#define START_BYTES_8BFR 2   // old not nec..
#define METADATA_WORDS_8BFR 4 // old not nec...

// Defines for the MEMSIC sensor
#define BYTES_META_DATA_ALL  28   // OLD not nec!!// 24; how much bytes contain the meta data of the radar & magnetic field sensor

// Defines for UART
//#define UART_TX_BUFF_SIZE 256 // nec?
//#define UART_RX_BUFF_SIZE 256

