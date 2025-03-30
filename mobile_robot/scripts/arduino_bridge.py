#!/usr/bin/env python3

import serial
from pynput.keyboard import Controller

keyboard = Controller()
arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust the port as needed

try:
    while True:
        data = arduino.readline().decode().strip()
        if data:
            keyboard.press(data)
            keyboard.release(data)
except KeyboardInterrupt:
    arduino.close()
    print("Exiting...")