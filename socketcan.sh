#!/bin/bash

#load gps
sudo chmod 777 /dev/ttyUSB0

# Load CAN kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Configure CAN interface (example for can1)
sudo ip link set can1 up type can bitrate 500000 dbitrate 5000000 sjw 4 restart-ms 1000 berr-reporting on fd on loopback off
