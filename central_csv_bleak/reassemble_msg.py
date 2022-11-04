from dataclasses import dataclass # requires Python > 3.7
from dataclass_csv import DataclassWriter
import time
import csv
import math

@dataclass
class Radar_Metadata:
    start_m: float = 0.0
    length_m: float = 0.0
    data_length: int = 0
    stitch_count: int = 0
    step_length_m: float = 0.0
    depth_lowpass_cutoff_ratio: float = 0.0

# TODO more flexible solution for the magnetometer's metadata    
@dataclass
class MMC_Metadata:
    offsetX0: int = 0
    offsetY0: int = 0
    offsetZ0: int = 0

    offsetX1: int = 0
    offsetY1: int = 0
    offsetZ1: int = 0

    offsetX2: int = 0
    offsetY2: int = 0
    offsetZ2: int = 0

    offsetX3: int = 0
    offsetY3: int = 0
    offsetZ3: int = 0

    offsetX4: int = 0
    offsetY4: int = 0
    offsetZ4: int = 0            

    offsetX5: int = 0
    offsetY5: int = 0
    offsetZ5: int = 0


class Reassemble_Ble_Msg:
    def __init__(self, bytes_msg, bytes_all_msg):
        self.buffer = b""  
        self.msg_counter = 0# +1 if a new message comes in
        self.bytes_all_msg = bytes_all_msg  # How much bytes has got all messages together?
        self.bytes_msg = bytes_msg          # bytes per message
        self.number_of_msgs = 0             # number of expected messages

        if (bytes_all_msg % bytes_msg) == 0:
            self.number_of_msgs = bytes_all_msg / bytes_msg
        else:
            self.number_of_msgs = math.ceil(bytes_all_msg / bytes_msg)

    def reassemble(self, msg):
        self.buffer += msg    # the N byte data frame is appended to the buffer until the whole metadata is received
        self.msg_counter += 1

        if self.msg_counter == self.number_of_msgs:
            ret = self.buffer
            self.buffer = b""       # reset buffer
            self.msg_counter = 0    # reset counter
            return True, ret
        else:
            return False, 0

    def update_bytes_all_msg(self, bytes_all_msg):   
        self.bytes_all_msg = bytes_all_msg

        if (self.bytes_all_msg % self.bytes_msg) == 0:
            self.number_of_msgs = self.bytes_all_msg / self.bytes_msg
        else:
            self.number_of_msgs = math.ceil(self.bytes_all_msg / self.bytes_msg)            

        print("self.number_of_msgs ", self.number_of_msgs )

    def get_counter(self):
        return self.msg_counter

    def info(self, who):
        print("I'm ", who)
        #print("Buffer:", self.buffer)
        print("MSG counter:", self.msg_counter)
        print("Number of messages:", self.number_of_msgs)    
        print("--------------------------------")


class RX_Notifications:
    def __init__(self, radar_metadata_csv, mmc_metadata_csv, radar_signal_csv, ATT_MTU, number_mgf_sensors, number_lidar_sensors):
        # Flags of the incoming messages
        self.MSG_SEND_METADATA  = 0xB0
        self.MSG_SEND_RADAR     = 0xA0
        self.MSG_SEND_MGF       = 0xA1
        self.MSG_SEND_LIDAR     = 0xA2

        # CSV-file attributes
        self.radar_metadata_csv = radar_metadata_csv
        self.mmc_metadata_csv = mmc_metadata_csv
        self.radar_signal_csv = radar_signal_csv
        self.path_measurements_csv = ""
        self.path_mmc_csv = ""
        self.absolute_number_measurements = 0
        self.length_csv_row = 0 # how much elements contains a csv-row? set later

        # Objects which saves the metadata
        self.metadata_radar = Radar_Metadata()
        self.metadata_mmc = MMC_Metadata()

        # Constants for reassambling the received data
        self.NUMBER_MGF_SENSORS   = number_mgf_sensors
        self.NUMBER_LIDAR_SENSORS = number_lidar_sensors

        # Define how much bytes a message has
        self.bytes_nus = ATT_MTU-3 # data frame of the NUS contains ATT_MTU-3 bytes
        self.bytes_general_information = 8
        self.bytes_metadata            = self.bytes_general_information + 9 * self.NUMBER_MGF_SENSORS
        self.bytes_mgf = self.bytes_general_information + 11 * self.NUMBER_MGF_SENSORS 
        self.bytes_lidar               = self.bytes_general_information + 6 * self.NUMBER_LIDAR_SENSORS      

        # Prepare buffers
        self.metadata_buffer = Reassemble_Ble_Msg(self.bytes_nus, self.bytes_metadata); # not nec
        self.radar_buffer = Reassemble_Ble_Msg(self.bytes_nus, 1); # will be updated by metadata
        self.mgf_buffer = Reassemble_Ble_Msg(self.bytes_nus, self.bytes_mgf); # not nec
        self.lidar_buffer = Reassemble_Ble_Msg(self.bytes_nus, self.bytes_lidar); # not nec       

        # Variables for the RADAR sensor
        self.mag = [0] * 1
        self.phase = [0] * 1

        # Variables for the MGF sensor
        self.mgf_x = [0] * self.NUMBER_MGF_SENSORS 
        self.mgf_y = [0] * self.NUMBER_MGF_SENSORS
        self.mgf_z = [0] * self.NUMBER_MGF_SENSORS
        self.die_temp = [0.0] * self.NUMBER_MGF_SENSORS

        # Variables for the LiDAR sensor
        self.dist = [0] * self.NUMBER_LIDAR_SENSORS 
        self.strength = [0] * self.NUMBER_LIDAR_SENSORS 
        self.temp_lidar = [0] * self.NUMBER_LIDAR_SENSORS         

    def process_metadata_msg(self, byte_msg):     
        # time not available without real time clock!
        # TODO setup real time clock!
        tm = time.strftime('%Y-%m-%d_%H:%M:%S_') # time whole buffer received/built
        temp_radar_metadata_csv = "./metadata_csv/" + tm + self.radar_metadata_csv + ".csv" # concatenation of  the constructor's file name and the time
        temp_mmc_metadata_csv = "./metadata_csv/" + tm + self.mmc_metadata_csv + ".csv"

        # RADAR's metadata
        self.metadata_radar.depth_lowpass_cutoff_ratio = byte_msg[1]/100.0
        self.metadata_radar.start_m = (byte_msg[2]/100.0)-0.7
        self.metadata_radar.length_m = (byte_msg[3]/10.0)

        self.metadata_radar.data_length = int.from_bytes(byte_msg[4:6], byteorder='big', signed=False) 
        self.metadata_radar.stitch_count = int.from_bytes(byte_msg[6:8], byteorder='big', signed=False)
        self.metadata_radar.step_length_m = int.from_bytes(byte_msg[8:10], byteorder='big', signed=False)/1000000.0

        # Setup variables and additinal information for the radar signal
        self.radar_buffer.update_bytes_all_msg(self.bytes_general_information+4*self.metadata_radar.data_length)
        self.mag = [0] * self.metadata_radar.data_length
        self.phase = [0] * self.metadata_radar.data_length

        # Memsic's metadata: all 6 sensors
        self.metadata_mmc.offsetX0 = int.from_bytes(byte_msg[10:13], byteorder='big', signed=False)
        self.metadata_mmc.offsetY0 = int.from_bytes(byte_msg[13:16], byteorder='big', signed=False)
        self.metadata_mmc.offsetZ0 = int.from_bytes(byte_msg[16:19], byteorder='big', signed=False)

        # self.metadata_mmc.offsetX1 = int.from_bytes(byte_msg[19:22], byteorder='big', signed=False)
        # self.metadata_mmc.offsetY1 = int.from_bytes(byte_msg[22:25], byteorder='big', signed=False)
        # self.metadata_mmc.offsetZ1 = int.from_bytes(byte_msg[25:28], byteorder='big', signed=False) 

        # self.metadata_mmc.offsetX2 = int.from_bytes(byte_msg[28:31], byteorder='big', signed=False)
        # self.metadata_mmc.offsetY2 = int.from_bytes(byte_msg[31:34], byteorder='big', signed=False)
        # self.metadata_mmc.offsetZ2 = int.from_bytes(byte_msg[34:37], byteorder='big', signed=False) 

        # self.metadata_mmc.offsetX3 = int.from_bytes(byte_msg[37:40], byteorder='big', signed=False)
        # self.metadata_mmc.offsetY3 = int.from_bytes(byte_msg[40:43], byteorder='big', signed=False)
        # self.metadata_mmc.offsetZ3 = int.from_bytes(byte_msg[43:46], byteorder='big', signed=False) 

        # self.metadata_mmc.offsetX4 = int.from_bytes(byte_msg[46:49], byteorder='big', signed=False)
        # self.metadata_mmc.offsetY4 = int.from_bytes(byte_msg[49:52], byteorder='big', signed=False)
        # self.metadata_mmc.offsetZ4 = int.from_bytes(byte_msg[52:55], byteorder='big', signed=False) 

        # self.metadata_mmc.offsetX5 = int.from_bytes(byte_msg[55:58], byteorder='big', signed=False)
        # self.metadata_mmc.offsetY5 = int.from_bytes(byte_msg[48:61], byteorder='big', signed=False)
        # self.metadata_mmc.offsetZ5 = int.from_bytes(byte_msg[61:64], byteorder='big', signed=False) 

        # Adjust length of the radar array
        self.radar_signal = [0] * self.metadata_radar.data_length

        # Save metadata as csv-file
        with open(temp_radar_metadata_csv, "w") as f:
            w = DataclassWriter(f, [self.metadata_radar], Radar_Metadata) 
            w.write()   
        with open(temp_mmc_metadata_csv, "w") as f:
            w = DataclassWriter(f, [self.metadata_mmc], MMC_Metadata) 
            w.write()   

        # Prepare csv-file for the measurements       
        csv_header = ["time", "occupied_radar","occupied_mgf","occupied_lidar"]

        for k in range(self.NUMBER_MGF_SENSORS):
            csv_header.append("mgf_x"+str(k))
            csv_header.append("mgf_y"+str(k))
            csv_header.append("mgf_z"+str(k))
            csv_header.append("die_temp"+str(k))

        for k in range(self.NUMBER_LIDAR_SENSORS):
            csv_header.append("dist"+str(k))
            csv_header.append("strength"+str(k))
            csv_header.append("lidar_temp"+str(k))     

        for k in range(self.metadata_radar.data_length):
            csv_header.append("mag"+str(k))

        for k in range(self.metadata_radar.data_length):
            csv_header.append("phase"+str(k))

        self.length_csv_row = len(csv_header)

        tm = time.strftime('%Y-%m-%d_%H:%M:%S_') # time for filename
        self.path_measurements_csv = "./signals_csv/" + tm + self.radar_signal_csv + ".csv"        

        # Write header to file
        with open(self.path_measurements_csv, 'w') as f:
            write = csv.writer(f)  
            write.writerow(csv_header)          

    def process_radar_msg(self, data):

        reassembled, byte_msg = self.radar_buffer.reassemble(data)    
        if reassembled:
            # General informations
            occupied = byte_msg[1]
            timestamp = int.from_bytes(byte_msg[2:6], byteorder='big', signed=False) 
            time_offset_ms = int.from_bytes(byte_msg[6:8], byteorder='big', signed=False) 
            timestamp = timestamp + (time_offset_ms/1000.0) # add offset to the timestamp, but convert in second first, strange bug offset_rx = 2*offset_real, but in C & python everything seem right??                    
            csv_row = [str(timestamp), str(occupied), "0", "0"]
                    
            # Zero filling csv for not used columns, before the radar data
            for k in range(self.length_csv_row - (self.metadata_radar.data_length*2) - 4):
                csv_row.append("0") 
                    
            # Reassemble real and imaginary part of the radar signal
            for k in range(self.metadata_radar.data_length):
                self.mag[k] = float(int.from_bytes(byte_msg[(8+2*k):(10+2*k)], byteorder='big', signed=False)/1000.0)
                self.phase[k] = float(int.from_bytes(byte_msg[(8+2*k+2*self.metadata_radar.data_length):(10+2*k+2*self.metadata_radar.data_length)], byteorder='big', signed=True)/10000.0)
                csv_row.append(self.mag[k]) 
                    
            # Add phase to row
            for k in range(self.metadata_radar.data_length):
                csv_row.append(self.phase[k])
                        
            # Append row to file
            with open(self.path_measurements_csv, "a") as fw:
                write = csv.writer(fw)  
                write.writerow(csv_row)              


    def process_mgf_msg(self, byte_msg):
    
        # General informations
        occupied = byte_msg[1]
        timestamp = int.from_bytes(byte_msg[2:6], byteorder='big', signed=False) 
        time_offset_ms = int.from_bytes(byte_msg[6:8], byteorder='big', signed=False) 
        timestamp = float(timestamp) + (time_offset_ms/1000.0) # add offset to the timestamp, but convert in second first 
        csv_row = [str(timestamp), "0", str(occupied), "0"] 
        
        # Reassemble the mgf x,y,z ans the sensor's die temperature
        for k in range(self.NUMBER_MGF_SENSORS):
            self.mgf_x[k] = int.from_bytes(byte_msg[(3*k+8):(3*k+11)], byteorder='big', signed=False)    
            self.mgf_y[k] = int.from_bytes(byte_msg[(3*k+8+3*self.NUMBER_MGF_SENSORS):(3*k+11+3*self.NUMBER_MGF_SENSORS)], byteorder='big', signed=False)  
            self.mgf_z[k] = int.from_bytes(byte_msg[(3*k+8+6*self.NUMBER_MGF_SENSORS):(3*k+11+6*self.NUMBER_MGF_SENSORS)], byteorder='big', signed=False)  
            self.die_temp[k] = (int.from_bytes(byte_msg[(2*k+8+9*self.NUMBER_MGF_SENSORS):(2*k+10+9*self.NUMBER_MGF_SENSORS)], byteorder='big', signed=False)-75.0)/10.0         
            if self.die_temp[k] >= 75.0:
                self.die_temp[k] = 0.0

            csv_row.append(str(self.mgf_x[k]))            
            csv_row.append(str(self.mgf_y[k]))  
            csv_row.append(str(self.mgf_z[k]))  
            csv_row.append(str(self.die_temp[k])) 
           
        # Zero filling csv for not used columns, TODO is this correct??
        for k in range(self.length_csv_row - (self.NUMBER_MGF_SENSORS*4) - 4):
            csv_row.append("0")         
        
        # Append row to file
        with open(self.path_measurements_csv, "a") as fw:
            write = csv.writer(fw)  
            write.writerow(csv_row)         

    def process_lidar_msg(self, byte_msg):

        # General informations
        occupied = byte_msg[1]
        timestamp = int.from_bytes(byte_msg[2:6], byteorder='big', signed=False) 
        time_offset_ms = int.from_bytes(byte_msg[6:8], byteorder='big', signed=False) 
        timestamp = float(timestamp) + (time_offset_ms/1000.0) # add offset to the timestamp, but convert in second first
        csv_row = [str(timestamp), "0", "0", str(occupied)]

        # Zero filling csv for not used columns, before the lidar data
        for k in range(self.length_csv_row - (self.NUMBER_LIDAR_SENSORS*3) - (self.metadata_radar.data_length*2) - 4):
            csv_row.append("0")  

        # Reassemble distance, strengt and lidar temperature
        for k in range(self.NUMBER_LIDAR_SENSORS):
            self.dist[k] = int.from_bytes(byte_msg[(8+2*k):(10+2*k)], byteorder='big', signed=False)
            self.strength[k] = int.from_bytes(byte_msg[(8+2*k+2*self.NUMBER_LIDAR_SENSORS):(10+2*k+2*self.NUMBER_LIDAR_SENSORS)], byteorder='big', signed=False)
            self.temp_lidar[k] = int.from_bytes(byte_msg[(8+2*k+4*self.NUMBER_LIDAR_SENSORS):(10+2*k+4*self.NUMBER_LIDAR_SENSORS)], byteorder='big', signed=False)   
            
            csv_row.append(str(self.dist[k]))            
            csv_row.append(str(self.strength[k]))  
            csv_row.append(str(self.temp_lidar[k]))  
            
        # Zero filling csv for not used columns, after the lidar data
        for k in range(self.length_csv_row - (self.NUMBER_LIDAR_SENSORS*3) - (self.NUMBER_MGF_SENSORS*4) - 4):
            csv_row.append("0")

        # Append row to file
        with open(self.path_measurements_csv, "a") as fw:
            write = csv.writer(fw)  
            write.writerow(csv_row)               

    def merge_rows_csv(self):
        print("mrows")          
