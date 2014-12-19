import serial
import time
import sys
import os
import math
import random
import binascii


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
	data.append(0x0A) 						#Packet Bite Length
	data.append(0xFE) 						#Servo ID (pID) - 0xFE = ALL
	data.append(0x03) 						#CMD
	data.append(0x34) 						#Data - Command Address
	data.append(0x01) 						#Data - Command Length
	data.append(0x60) 						#Data - Write error = 0
	dataToSend = []
	dataToSend.append('0A') 				#Packet Bite Length
	dataToSend.append('FE') 				#Servo ID (pID) - 0xFE = ALL
	dataToSend.append('03') 				#CMD
	dataToSend.append('34') 				#Data - Command Address
	dataToSend.append('01') 				#Data - Command Length
	dataToSend.append('60') 				#Data - Write error = 0
	sendData(data,dataToSend);

	
# Send data to servo 
def sendData(data,dataToSend):	
	dataToSend.insert(0,'FF')						# Add header
	dataToSend.insert(1,'FF')						# Add header
	dataArrayLength=len(data)						# Get data Array Length
	ck1=checksum1(data,dataArrayLength) 			# Generate checksum 1
	ck2=checksum2(ck1)								# Generate checksum 2
	ck1Format = '%02X' % ck1
	ck2Format = '%02X' % ck2
	dataToSend.insert(5,ck1Format)					# Insert checksum 1
	dataToSend.insert(6,ck2Format)					# Insert checksum 2
	stringToSend = ""
	for i in range(len(dataToSend)):
		stringToSend = stringToSend + "\\x" + dataToSend[i]
	#print "Attempting:"
	#print type(stringToSend)
	print stringToSend
	serPort.write(stringToSend.decode('string-escape'))
	#print "Working:"
	#working2 = '\xFF\xFF\x0A\xFE\x03\xA2\x5C\x34\x01\x60'
	#working = '\xFF\xFF\x0C\x00\x06\x96\x68\xC8\x56\x02\x00\x01'
	#print working
	#serPort.write(working)
	#print working2
	#serPort.write(working2)
	print " "


def start():
	clearError()


	
start()

var = 0

while var == 1 :
	operatingTime = random.randint(1,254)
	positionGoal = random.randint(21,1002)
	#print operatingTime
	singleServoMove(0x01,positionGoal,operatingTime,0x01)
	time.sleep(3)
	#serPort.write("\xFF\xFF\x0C\x01\x06\x96\x68\xC8\x56\x02\x00\x01")
	#time.sleep(2)
