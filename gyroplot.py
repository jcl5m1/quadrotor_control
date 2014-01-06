import numpy as np
import matplotlib.pyplot as plt
import sys, os, serial, threading
import pygame
from pygame.locals import *
from collections import deque


red = pygame.Color(255,0,0)
green = pygame.Color(0,255,0)
yellow = pygame.Color(255,255,0)
cyan = pygame.Color(0,255,255)
blue = pygame.Color(0,0,255)
black = pygame.Color(0,0,0)
white = pygame.Color(255,255,255)

pygame.init()
fpsClock = pygame.time.Clock()

width = 800
hieght = 800

windowSurfaceObj = pygame.display.set_mode((width,hieght))
pygame.display.set_caption('Python Plot')

def convertBytes(b1, b2):
	value = (b1<<8) + b2
	if(value > 32768):
		return value - 65536
	return value

def parseData():
	d = ord(ser.read())
	if(d != 170):
		return;
	x = convertBytes(ord(ser.read()),ord(ser.read()))
	y = convertBytes(ord(ser.read()),ord(ser.read()))
	z = convertBytes(ord(ser.read()),ord(ser.read()))
	return x,y,z
	
def convertScale(v):
	return hieght/2 - v/10
 
# class that holds analog data for N samples
class AnalogData:
	# constr
	def __init__(self, maxLen):
		self.ax = deque([0.0]*maxLen)
		self.ay = deque([0.0]*maxLen)
		self.az = deque([0.0]*maxLen)
		self.maxLen = maxLen
		
	#compute statistics
	def computeStats(self,buf):
		mu = 0.0
		sig = 0.0
		for i in buf:
			mu += i
		mu /= len(buf)
		for i in buf:
			sig += (i-mu)*(i-mu)
		sig /= len(buf)-1
		return mu,sig
		
	def computeAllStats(self):
		print "x: ",self.computeStats(self.ax)
		print "y: ",self.computeStats(self.ay)
		print "z: ",self.computeStats(self.az)	

	# ring buffer
	def addToBuf(self, buf, val):
		if len(buf) < self.maxLen:
			buf.append(val)
		else:
			buf.pop()
			buf.appendleft(val)
						
	# add data
	def add(self, data):
		assert(len(data) == 3)
		self.addToBuf(self.ax, data[0])
		self.addToBuf(self.ay, data[1])
		self.addToBuf(self.az, data[2])


port = "/dev/ttyACM0"
baud = 57600
print "Openning",port,"at",baud,"..."
ser = serial.Serial(port, baud, timeout=1)

xCursor = 0;

px2 = (0,0)
py2 = (0,0)
pz2 = (0,0)

observationCount = 1000
analogData = AnalogData(observationCount)
dataCount = 0

while True:

	if ser.isOpen():
		data = parseData();
		print "Data",dataCount,":",data
		analogData.add(data)
		dataCount += 1
		if(dataCount == observationCount):
			analogData.computeAllStats()
			pygame.quit()
			sys.exit()
		px1 = px2
		py1 = py2
		pz1 = pz2
		px2 = (xCursor,convertScale(data[0]));
		py2 = (xCursor,convertScale(data[1]));
		pz2 = (xCursor,convertScale(data[2]));
		if(xCursor == 0):
			px1 = px2
			py1 = py2
			pz1 = pz2
		pygame.draw.rect(windowSurfaceObj, black,(xCursor,0,10,hieght))
		pygame.draw.line(windowSurfaceObj, white, (xCursor+1,0),(xCursor+1,hieght),1)
		pygame.draw.line(windowSurfaceObj, red, px1,px2,1)
		pygame.draw.line(windowSurfaceObj, green, py1,py2,1)
		pygame.draw.line(windowSurfaceObj, blue, pz1,pz2,1)
		xCursor += 1
		if(xCursor >= width):
			xCursor = 0
	for event in pygame.event.get():
		if event.type == QUIT:
			pygame.quit()
			sys.exit()
		if event.type == KEYDOWN:
			if event.key == K_ESCAPE:
				pygame.event.post(pygame.event.Event(QUIT))
	pygame.display.update()

ser.close()

	

