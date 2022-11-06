# Repository: B.Eng. Thesis

## A Bluetooth Low Energy based Data Generating System for the Development of Parking Detection Algorithms using RADAR and Magnetic Field Sensors

This repository contains the data sets and the source code of the author's Bachelor of Engineering Thesis.

## Measurements
- xx_measurement_msg_loss.csv: file where some rows are incomplete, e.g., the RADAR and LiDAR signals are given, but the magnetometer data misses.
- xx_measurement_clean.csv: file where the incomplete rows are deleted

## XM122
- the firmware of the Acconner XM122
- this is a microcontroller and pulsed RADAR, based on the Nordic nRF52840
- the source code contains the Acconeer's SDK v2.11.0 for the XM122
- the SDK is based on the Nordic nRF5 SDK v15.3.0
- the SDK is not modified, except for the sdk_config.h, the files which are associated to settings of Segger Embedded Studio (used IDE for the project) and custom files are added
- custom files are:
  - /XM122/include:
    - memsic_mmc5983ma.h
    - pd_ble_messages.h
    - pd_project_includes.h
    - pd_project_defines.h (not used anymore)
    - tfmini_plus.h
    - tfmini_plus_uart.h (highly incomplete, but not necessary here)
  - /XM122/source:
    - main.c
    - memsic_mmc5983ma.c
    - pd_ble_messages.c
    - tfmini_plus.c
    - tfmini_plus_uart.c (highly incomplete, but not necessary here)

## Gateway
- folder: central_csv_bleak
- hardware based on the Raspberry Pi Computing Module 4
- Python source code is excecuted by the crontab after a reboot of the RPI
- central_csv_bleak.py: main file
- iocontrol.py: GPIO handling, i.e., control LEDs, handle switches / inputs
- pd_sensor_cnfg.py: build configuration messages for the XM122
- reassemble_msg.py: reassemble sliced messages, save signals as csv-file
- services_characteristics.txt: Nordic UART Service information, e.g. UUID
