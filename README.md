# Repository: B.Eng. Thesis

## A Bluetooth Low Energy based Data Generating System for the Development of Parking Detection Algorithms using RADAR and Magnetic Field Sensors

This repository contains the data sets and the source code of the author's Bachelor of Engineering Thesis.

## Measurements
- xx_measurement_msg_loss.csv: file where some rows are incomplete, e.g., the RADAR and LiDAR signals are given, but the magnetometer data misses.
- xx_measurement_clean.csv: file where the incomplete rows are deleted

## XM122
- folder: sensorsystem
- the firmware of the Acconner XM122
- this is a microcontroller and pulsed RADAR, based on the Nordic nRF52840
- the source code contains the Nordic SDK 15.3 and Acconeer's SDK for the XM122
- the SDKs are not modified, except for the sdk_config.h and the files which are associated to settings of Segger Embedded Studio (used IDE for the project)
- main file and other libraries: sensorsystem/nRF5_SDK_15.3.0_59ac345/xm122/source
- header files: sensorsystem/nRF5_SDK_15.3.0_59ac345/xm122/include

## Gateway
- folder: central_csv_bleak
- hardware based on the Raspberry Pi Computing Module 4
- Python source code is excecuted by the crontab after a reboot of the RPI
- central_csv_bleak.py: main file
- iocontrol.py: GPIO handling, i.e., control LEDs, handle switches / inputs
- pd_sensor_cnfg.py: build configuration messages for the XM122
- reassemble_msg.py: reassemble sliced messages, save signals as csv-file
- services_characteristics.txt: Nordic UART Service information, e.g. UUID
