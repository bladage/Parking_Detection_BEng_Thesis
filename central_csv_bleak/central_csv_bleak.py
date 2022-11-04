#
# This script implements a central role (BLE terminology) which collects data
# from a sensor system (peripheral role) to develope Parking Detection algorithms.
# The sensor system contains a RADAR, magnetometers and LiDARs. This script 
# setups all kind of used sensors and receives their data which are saved
# as an csv-file.
# Bluetoothly speaking, the script is written from the perspective of the 
# GATT-layer. It uses the bleak BLE interface for Python, see
# 		https://bleak.readthedocs.io 
#	more good examples:	 
#		https://gopro.github.io/OpenGoPro/tutorials/python/connect-ble
#		https://nabeelvalley.co.za/docs/iot/bluetooth-intro/ 
#		https://github.com/coyt/PythonNUS/blob/master/main.py            
#
# Bennet L. Ladage
# 2022-10-11
#
# B.Eng. Thesis
# South Westphalia University of Applied Sciences 
# Meschede
#

import asyncio
from bleak import BleakClient

from dataclasses import dataclass # requires Python > 3.7
from dataclass_csv import DataclassWriter
import time
import csv
import math
import RPi.GPIO as GPIO

import os
import sys
import psutil
import logging

# Own files
from iocontrol import IO_Control
from reassemble_msg import Reassemble_Ble_Msg, RX_Notifications
from pd_sensor_cnfg import Sensor_Configuration, NUS_GATT_Characteristics

from decimal import *
import numpy as np
   
def restart_program():
    """
    Restarts the current program, with file objects and descriptors cleanup
    """
    try:
        p = psutil.Process(os.getpid())
        for handler in p.open_files() + p.connections():
            os.close(handler.fd)
    except Exception as e:
        logging.error(e)

    python = sys.executable
    os.execl(python, python, *sys.argv)
    
#nec?
# class Event_ts(asyncio.Event):
    # """Custom event loop class for the sensor().
    # :param asyncio: None
    # :type asyncio: Event_ts inherit asyncio.Event functions.
    # """

    # def clear(self):     
        # self._loop.call_soon_threadsafe(super().clear)

    # def set(self):    
        # self._loop.call_soon_threadsafe(super().set)       

async def main():

	#									 #
	# Configuration of the Sensor System #
	#									 #
	
	# GATT information about the Nordic UART Service (NUS) and the BLE address of the Parking Detection (PD) sensor system
	nus_chars = NUS_GATT_Characteristics("C0:55:35:DA:B6:1D")	

	# General Settings
	sleep_period = 0
	number_mgf_sensors        = 1  # TODO: this info has to be appended to the metadata!
	number_lidar_sensors      = 2  # TODO: save as metadata!
			
	# RADAR Settings
	sample_period_r = 400        # sampling period RADAR
	start_cm = 8                  # start range RADAR
	length_cm = 50                # start_m+length_m=max. range RADAR
	downsampling_factor = 1       # set resolution of the RADAR's range axis: 1=0.5mm, 2=1mm, 4=2mm
	profile = 1                   # lower number=shorter wavelet, allowed values = {1, 2, 3, 4, 5}
	hwaas = 10                    # hardware accelearated average samples, [1; 63]
	depth_lowpass = 0.6           # Cutoff frequency ratio for distance domain low-pass filter of data, [0.0; 0.5]; 0.6 deactivates (TODO does this work?)  
			
	# Magnetic Field (mgf) Sensor Settings
	sample_period_mgf = 400		# sampling period mgf sensor
	filter_bandwidth = 800          # duration for each magnetic field measurement: 100=8.0ms, 200=4.0ms, 400=2.0ms, 800 Hz = 0.5 ms
	periodic_set_operation = 0      # Enable automatic SET of the magnetic field sensor, i.e., if a strong magnetic field (> 10 Gauss) is applied, the sensor characteristics are changed. This resets the characteristics. 0=on, 1=off
	repetition_number_pso = 2000    # How much measurements has to be passed that a SET operation is started automatically? Possible numbers = {1, 25, 75, 100, 250, 500, 1000, 2000}.
			
	# LiDAR Settings
	sample_period_lid = 400
	f_sample_internal_lid = 100	
	
	# File names of the csv files
	radar_metadata_csv = "radar_metad"   # file name for the RADAR's metadata
	mmc_metadata_csv = "mmc_metad"       # file name for the MMC's metadata
	data_csv = "measurements"            # file name measurements, i.e., RADAR, LiDARs and magnetometers

	# Control object for the switch and the LEDs
	io = IO_Control(12, 16, 18, 10); # LED0=Pin12, LED1=Pin16, LED2=Pin18, SWITCH=Pin10
	
	# Build config message
	s_config = Sensor_Configuration(sleep_period, sample_period_r, start_cm, length_cm, downsampling_factor, profile, f_sample_internal_lid, hwaas, depth_lowpass, sample_period_mgf, filter_bandwidth, periodic_set_operation, repetition_number_pso, sample_period_lid)
	nus_setup_msg = s_config.setup_msg()	
	
	# Object which handles incoming notifications
	rx = RX_Notifications(radar_metadata_csv, mmc_metadata_csv, data_csv, nus_chars.att_mtu, number_mgf_sensors, number_lidar_sensors)
	
	# Define an synchronization event which is used to wait for the notification MSG_SEND_METADATA (XM122 --> Raspi)
	event = asyncio.Event()
	
	# Event for scheduling callbacks??
	#stopEvent = Event_ts() 
	
	#
	# This event handler processes incoming notifications from the XM122, i.e., it receives data and saves it as a csv-file
	#
	def notification_handler(handle: int, data: bytearray):
		
		if data[0] == rx.MSG_SEND_RADAR or rx.radar_buffer.msg_counter > 0:
			rx.process_radar_msg(data)
			
		elif data[0] == rx.MSG_SEND_MGF:	
			rx.process_mgf_msg(data)
			
		elif data[0] == rx.MSG_SEND_LIDAR:
			rx.process_lidar_msg(data)
			
		elif data[0] == rx.MSG_SEND_METADATA:						
			rx.process_metadata_msg(data)
		else:
			print("<Warning> Received unkown data format!")		
	
	#
	# Start the (endless) measurement series if the switch is in on position
	#
	while True:
		io.wait_switch_on(0.2)
		if io.is_switch_on():
			io.leds_off()
			break      	
	#
	# LED0 as an indicator that the connection is going to be established
	#
	io.led0_on()
	try:
		async with BleakClient(nus_chars.addr) as client:
			#
			# Show that connection was successfully established, i.e., switch LED1 on
			#
			print("<Info> Connected!")
			io.led0_off()
			io.led1_on()
						
			#
			# Enable Notifications, send the Sensor Configuration via the Request "MSG_SETUP_CONFIG" and request the metadata "MSG_SEND_METADATA"
			#
			try:
				await client.start_notify(nus_chars.tx_uuid, notification_handler)   # enable notifications
				await client.write_gatt_char(nus_chars.rx_uuid, nus_setup_msg, True) # MSG_SETUP_CONFIG; True leads to request
				await asyncio.sleep(4) 											     # give the nRF52840 time to process the setup
				await client.write_gatt_char(nus_chars.rx_uuid, bytearray.fromhex('B0'), True) # MSG_SEND_METADATA = 0xB0
				await event.wait()  												 # wait to receive the notification (MSG_SEND_METADATA form the XM122)
				await asyncio.sleep(4) 												 # give the nRF52840 time to start the timers which are started after tx the metadata
					
			except Exception as e:			
				print(f"<Error> {e} - The application is restarted now.")
				io.error(0.2)
				restart_program()			
			
			#
			# Now, we can receive the sensor data via the notification_handler above, hence endless loop
			#
			io.led1_off() # LED1 off
			io.led2_on()  # LED2 on, i.e., in the device is in the receiving data state
			await asyncio.sleep(1) 
			
			#while True:	
			await event.wait() # wait to receive the notification, this is such as an endless loop
			#await asyncio.Event().wait() # wait for an event that never happen, i.e., this is an endless loop
			
			await client.disconnect()
			io.leds_off() # all LEDs off, i.e., disconnected successfully
			
	except Exception as e:			
		print(f"<Error> {e} - The application is restarted now.")
		io.error(0.2)
		restart_program()
	finally:
		io.leds_off()		
		
# 
# RUN the Application
#
asyncio.run(main()) # Using asyncio.run() is important to ensure that device disconnects on KeyboardInterrupt or other unhandled exception.
