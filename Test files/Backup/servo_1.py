import serial
import time
import sys
import os
import math
import random


#print "Opening serial port"

serPort = serial.Serial('/dev/ttyAMA0', 115200, timeout=3.0)
print "Serial port open"


	
# checksum1
def checksum1(data, lenghtString):
	buffer = 0	
	for x in range(0, lenghtString):
		buffer = buffer ^ data[x]
	return buffer&0xFE;

# checksum2
def checksum2(XOR):
	return (~XOR)&0xFE

# Clear Error
def clearError():
	data = []
	data.append(0x0B) 				#Packet Bite Length
	data.append(0xFE) 				#Servo ID (pID) - 0xFE = ALL
	data.append(0x03) 				#CMD
	data.append(0x30) 				#Data - Command Address
	data.append(0x02) 				#Data - Command Length
	data.append(0x00) 				#Data - Write error = 0
	data.append(0x00) 				#Data - Write detail error=0
	dataArrayLength=len(data)		#data Array Length
	#print data
	ck1=checksum1(data,dataArrayLength) # Generate checksum 1
	ck2=checksum2(ck1)					# Generate checksum 2
	#print ck1
	#print hex(ck1)
	#print ck2
	#print hex(ck2)
	sendData(data);

# Torque On
def torqueOn():
	data = []
	data.append(0x0A) 				#Packet Bite Length
	data.append(0xFE) 				#Servo ID (pID) - 0xFE = ALL
	data.append(0x03) 				#CMD
	data.append(0x34) 				#Data - Command Address
	data.append(0x01) 				#Data - Command Length
	data.append(0x60) 				#Data - Torque On
	dataArrayLength=len(data)		#data Array Length
	#print data
	ck1=checksum1(data,dataArrayLength) # Generate checksum 1
	ck2=checksum2(ck1)					# Generate checksum 2
	#print ck1
	#print hex(ck1)
	#print ck2
	#print hex(ck2)
	sendData(data);

# Set LED
def setLED(servoID,colorName):
	data = []
	data.append(0x0A) 				#Packet Bite Length
	data.append(servoID) 			#Servo ID (pID) - 0xFE = ALL
	data.append(0x03) 				#CMD
	data.append(0x35) 				#Data - Command Address
	data.append(0x01) 				#Data - Command Length
	data.append(colorName) 			#Data - LED Color 0x01=Green, 0x02 = Blue, 0x04 = Red
	dataArrayLength=len(data)		#data Array Length
	print data
	ck1=checksum1(data,dataArrayLength) # Generate checksum 1
	ck2=checksum2(ck1)					# Generate checksum 2
	#print ck1
	#print hex(ck1)
	#print ck2
	#print hex(ck2)
	sendData(data);

# Single Servo Move (I_JOG)
def singleServoMove(servoID,positionGoal,operatingTime,LEDcolorName):
	positionGoalMSB = positionGoal >> 8
	positionGoalLSB = positionGoal & 0xff
	data = []
	data.append(0x0C) 				#Packet Bite Length
	data.append(servoID) 			#Servo ID (pID) - 0xFE = ALL
	data.append(0x05) 				#CMD
	data.append(positionGoalLSB) 				#Data - JOG(LSB)
	data.append(positionGoalMSB) 				#Data - JOG(MSB)
	data.append(0x04) 				#Data - SET
	data.append(servoID) 			#Data - ID
	data.append(operatingTime) 		#Data - playtime
	dataArrayLength=len(data)		#data Array Length
	#print data
	ck1=checksum1(data,dataArrayLength) # Generate checksum 1
	ck2=checksum2(ck1)					# Generate checksum 2
	#print ck1
	#print hex(ck1)
	#print ck2
	#print hex(ck2)
	sendData(data);	

# Send data to servo 
def sendData(data):	
	test1('d')
	#print type(test1)
	data.insert(0,0xFF)					# Add header
	data.insert(1,0xFF)					# Add header
	dataArrayLength=len(data)			# Get data Array Length
	ck1=checksum1(data,dataArrayLength) # Generate checksum 1
	ck2=checksum2(ck1)					# Generate checksum 2
	data.insert(5,ck1)					# Insert checksum 1
	data.insert(6,ck2)					# Insert checksum 2
	print data
	print hex(data[1])
	stringToSend = ""
	for i in range(len(data)):
		byteFormat = '%02X' % data[i]
		#print byteFormat
		stringToSend = stringToSend + "\\x" + byteFormat
		#stringToSend = stringToSend + "\\x" + data[i]
	print "Attempting:"
	#print type(stringToSend)
	print stringToSend
	serPort.write(stringToSend)
	print "Working:"
	working2 = '\xFF\xFF\x0A\xFE\x03\xA2\x5C\x34\x01\x60'
	working = '\xFF\xFF\x0C\x00\x06\x96\x68\xC8\x56\x02\x00\x01'
	print working
	serPort.write(working)
	print working2
	serPort.write(working2)
	print " "


def start():
	clearError()
	print "... errors cleared"
	time.sleep(3)
	torqueOn()
	print "... torque on"
	time.sleep(3)
	time.sleep(3)
	singleServoMove(0x00,512,65,1)
	#singleServoMove(0x00,512,65,1)

# Start of application
start()