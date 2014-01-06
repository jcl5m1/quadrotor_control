import sys, os, serial, threading

port = "COM4"
baud = 57600
print "Openning",port,"at",baud,"..."
ser = serial.Serial(port, baud, timeout=1)
gyroDiv = int(50)
pitch = int(128)
yaw = int(128)
roll = int(128)
lift = 0;

while True:
	d = ord(ser.read())
	print d


while ser.isOpen():
	
	var = raw_input("power[0-9]: ")
	if(len(var) == 0):
		var =lift
		print lift
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
	
	if(var == 't'):
		ser.write('g')
		ser.write(chr(0))
		ser.write(chr(pitch))
		ser.write(chr(yaw))
		ser.write(chr(roll))
		ser.write(chr(gyroDiv))
		ser.write('t')
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
