import os 
import serial
import time
from math import *
ONE_BLOCK_Bytes = 32
ser = serial.Serial("COM9", 921600,timeout=25) 
res = ""

def waitfor(ser_dev,x):
    data = ""
    while data!=x:
        data = ser.read(len(x))
    print(x.decode())

waitfor(ser,b"Bootloader started!")
ser.write(b"FIRMWARE")
waitfor(ser,b"FIRMWAREOK")
my_file = "F303_firmware_2.bin"
size = os.path.getsize(my_file)
time.sleep(1)
ser.write((size).to_bytes(4))
print("sended ",(size).to_bytes(4))
waitfor(ser,b"SIZEOK")
data = ser.readline()
print(data)
packages = ceil(size / ONE_BLOCK_Bytes)
with open(my_file,"rb") as f:
    for i in range(packages):
        if i < (packages - 1):
            data = f.read(ONE_BLOCK_Bytes)
            data = data[0:4][::-1] + data[4:8][::-1] + data[8:12][::-1] + data[12:16][::-1] + data[16:20][::-1] + data[20:24][::-1] + data[24:28][::-1] + data[28:32][::-1]
            #print("to_send",data)
        else: 
            remains = packages*ONE_BLOCK_Bytes - size
            data = f.read(remains)
            data = data[0:4][::-1] + data[4:8][::-1] + data[8:12][::-1] + data[12:16][::-1] + data[16:20][::-1] + data[20:24][::-1] + data[24:28][::-1] + data[28:32][::-1]
            data += b'\0'*(ONE_BLOCK_Bytes-remains)
            #print("to_send_last")
            #print(data)
        res = ""
        while res != b"OK":
            time.sleep(1)
            ser.write(data)
            #print("Got",ser.read(ONE_BLOCK_Bytes))
            res = ser.read(2)
            if (i+1) % 10 ==0:
                print(res if res else "Err"," ",i+1, "/",packages)
            #print(data)
        #print("----------------------")
    print("SENDED ALL DATA")
while True:
    data = ser.read(1)
    print(data.decode(),end="")
