import time

class Sensor_Configuration:
    def __init__(self, sleep_period, sample_period_r, start_cm, length_cm, downsampling_factor, profile, fs_lidar, hwaas, depth_lowpass, sample_period_mgf, filter_bandwidth, periodic_set_operation, repetition_number_pso, sample_period_lid):
        # General Settings
        self.timestamp = 0
        self.sleep_period = sleep_period
        
        # RADAR Settings
        self.sample_period_r = sample_period_r
        self.start_cm = start_cm                             # start range RADAR
        self.length_cm = length_cm                           # start_m+length_m=max. range RADAR
        self.downsampling_factor = downsampling_factor       # set resolution of the RADAR's range axis: 1=0.5mm, 2=1mm, 4=2mm
        self.profile = profile                               # lower number=shorter wavelet, allowed values = {1, 2, 3, 4, 5}
        self.fs_lidar = fs_lidar                             # internal sampling frequency LiDAR
        self.hwaas = hwaas                                   # hardware accelearated average samples, [1; 63]
        self.depth_lowpass = depth_lowpass                   # cutoff frequency ratio for distance domain low-pass filter of data   
        
        # Magnetic Field (mgf) Sensor Settings
        self.sample_period_mgf = sample_period_mgf
        self.filter_bandwidth = filter_bandwidth             # duration for each magnetic field measurement: 100=8.0ms, 200=4.0ms, 400=2.0ms, 800 Hz = 0.5 ms
        self.periodic_set_operation = periodic_set_operation # Enable automatic SET of the magnetic field sensor, i.e., if a strong magnetic field (> 10 Gauss) is applied, the sensor characteristics are changed. This resets the characteristics. 0=on, 1=off
        self.repetition_number_pso = repetition_number_pso   # How much measurements has to be passed that a SET operation is started automatically? Possible numbers = {1, 25, 75, 100, 250, 500, 1000, 2000}.
        
        # LiDAR Settings
        self.sample_period_lid = sample_period_lid
        
        # Message flags
        self.MSG_SETUP_CONFIG        = 0xC0
        self.MSG_UPDATE_RADAR_CONFIG = 0xC1 # not used yet
        self.MSG_UPDATE_MGF_CONFIG   = 0xC2 # ~
        self.MSG_UPDATE_LIDAR_CONFIG = 0xC3 # ~
        self.MSG_UPDATE_TIMESTAMP    = 0xC4 # ~
        self.MSG_UPDATE_SLEEP_PERIOD = 0xC5 # ~
    
    def setup_msg(self):
        timestamp = int(time.time()) # seconds since 1970-01-01 UTC
        bit_mask =  0x000000FF
        nus_msg = bytearray(25)
        
        # Flag message
        nus_msg[0] = self.MSG_SETUP_CONFIG

        # 32 bit data, general setting
        nus_msg[1] = int((timestamp >> 24) & bit_mask)
        nus_msg[2] = int((timestamp >> 16) & bit_mask)
        nus_msg[3] = int((timestamp >> 8) & bit_mask)
        nus_msg[4] = int(timestamp & bit_mask)
        
        # 16 bit data, general setting 
        nus_msg[5] = int((self.sleep_period >> 8) & bit_mask)
        nus_msg[6] = int(self.sleep_period & bit_mask)
        
        # 16 bit data, radar
        nus_msg[7] = int((self.sample_period_r >> 8) & bit_mask)
        nus_msg[8] = int(self.sample_period_r & bit_mask)
        nus_msg[9] = int((self.fs_lidar >> 8) & bit_mask)
        nus_msg[10] = int(self.fs_lidar & bit_mask)
        
        # 16 bit data, mgf
        nus_msg[11] = int((self.sample_period_mgf >> 8) & bit_mask)
        nus_msg[12] = int(self.sample_period_mgf & bit_mask)
        nus_msg[13] = int((self.repetition_number_pso >> 8) & bit_mask)
        nus_msg[14] = int(self.repetition_number_pso & bit_mask)
        
        # 16 bit data, lidar
        nus_msg[15] = int((self.sample_period_lid >> 8) & bit_mask)
        nus_msg[16] = int(self.sample_period_lid & bit_mask)        
        
        # 8 bit data, radar
        nus_msg[17] = self.start_cm+7
        nus_msg[18] = self.length_cm
        nus_msg[19] = self.downsampling_factor
        nus_msg[20] = self.profile                                                   
        nus_msg[21] = self.hwaas                                   
        nus_msg[22] = int(self.depth_lowpass*100)
        
        # 8 bit data, mgf
        nus_msg[23] = int(self.filter_bandwidth/100)
        nus_msg[24] = self.periodic_set_operation
        return nus_msg
        
    def update_radar_msg(self):
        rn_avg = int(self.running_avg*100)
        bit_mask =  0x000000FF
        nus_msg = bytearray(11)
        
        # Flag message
        nus_msg[0] = self.MSG_UPDATE_RADAR_CONFIG
         
        # 16 bit data, radar
        nus_msg[1] = int((self.sample_period_r >> 8) & bit_mask)
        nus_msg[2] = int(self.sample_period_r & bit_mask)
        nus_msg[3] = int((rn_avg >> 8) & bit_mask)
        nus_msg[4] = int(rn_avg & bit_mask)      
        
        # 8 bit data, radar
        nus_msg[5] = self.start_cm+7
        nus_msg[6] = self.length_cm
        nus_msg[7] = self.downsampling_factor
        nus_msg[8] = self.profile                                                   
        nus_msg[9] = self.hwaas                                   
        nus_msg[10] = self.power_save_mode
        return nus_msg

    def update_mgf_msg(self):
        bit_mask =  0x000000FF
        nus_msg = bytearray(7)
        
        # Flag message
        nus_msg[0] = self.MSG_UPDATE_MGF_CONFIG
        
        # 16 bit data, mgf
        nus_msg[1] = int((self.sample_period_mgf >> 8) & bit_mask)
        nus_msg[2] = int(self.sample_period_mgf & bit_mask)
        nus_msg[3] = int((self.repetition_number_pso >> 8) & bit_mask)
        nus_msg[4] = int(self.repetition_number_pso & bit_mask)       
        
        # 8 bit data, mgf
        nus_msg[5] = int(self.filter_bandwidth/100)
        nus_msg[6] = self.periodic_set_operation
        return nus_msg   

    def update_lidar_msg(self):
        bit_mask =  0x000000FF
        nus_msg = bytearray(3)
        
        # Flag message
        nus_msg[0] = self.MSG_UPDATE_LIDAR_CONFIG
        
        # 16 bit data, lidar
        nus_msg[1] = int((self.sample_period_lid >> 8) & bit_mask)
        nus_msg[2] = int(self.sample_period_lid & bit_mask)        
        return nus_msg
    
    def update_timestamp(self):
        timestamp = int(time.time()) # seconds since 1970-01-01 UTC
        bit_mask =  0x000000FF
        nus_msg = bytearray(5)
        
        # Flag message
        nus_msg[0] = self.MSG_UPDATE_TIMESTAMP
        
        # 32 bit data
        nus_msg[1] = int((timestamp >> 24) & bit_mask)
        nus_msg[2] = int((timestamp >> 16) & bit_mask)
        nus_msg[3] = int((timestamp >> 8) & bit_mask)
        nus_msg[4] = int(timestamp & bit_mask)
        return nus_msg
        
    def update_sleep_period(self):
        bit_mask =  0x000000FF
        nus_msg = bytearray(3)
        
        # Flag message
        nus_msg[0] = self.MSG_UPDATE_SLEEP_PERIOD
        
        # 16 bit data, general setting 
        nus_msg[1] = int((self.sleep_period >> 8) & bit_mask)
        nus_msg[2] = int(self.sleep_period & bit_mask)
        return nus_msg
        
class NUS_GATT_Characteristics():
     def __init__(self, addr):
        self.addr = addr
        self.handle_service = 14
        self.uuid_service = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
        self.rx_uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
        self.tx_uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
        self.rx_handle_nus = 15
        self.tx_handle_nus = 17
        self.descriptor_uuid = "00002902-0000-1000-8000-00805f9b34fb"
        self.descriptor_handle = 19
        self.att_mtu = 247
