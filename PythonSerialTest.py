# -*- coding: utf-8 -*-
import serial

ser = serial.Serial('COM6', 9600)


print(1)
ser.write(bytes(1))
print(ser.read())
print(1)