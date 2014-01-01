import sys, os, serial, threading
ser = serial.Serial("COM4", 57600, timeout=1)
gyroDiv = int(50)
pitch = int(128)
yaw = int(128)
roll = int(128)

while True:
	var = raw_input("power[0-9]: ")
	if(len(var) == 0):
		var =0
	if(var == 'x'):
		break
		
		
	if(var == 'r'):
		ser.write('g')
		ser.write(chr(0))
		ser.write(chr(pitch))
		ser.write(chr(yaw))
		ser.write(chr(roll))
		ser.write(chr(gyroDiv))
		ser.write('r')
		continue
	
		
	if(int(var) > 255):
		continue
		
	lift = int(var)
		
	ser.write('g')
	ser.write(chr(lift))
	ser.write(chr(pitch))
	ser.write(chr(yaw))
	ser.write(chr(roll))
	ser.write(chr(gyroDiv))
	ser.write('s')
	
ser.close()
