# -*- coding: utf-8 -*-
import serial

ser = serial.Serial('COM6', 9600)
connected=False
while not connected:
    serin=ser.read()
    connected = True

ser.write( bytes([1]))
print(1)
