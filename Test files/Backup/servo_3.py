# Robot information
#
# Coxa Length 	42mm
# Femur Length	70mm
# Tibia Length	60mm

import serial
import time
import sys
import os
import math
import random

#print "Opening serial port"
serPort = serial.Serial('/dev/ttyAMA0', 115200, timeout=3.0)
print "Serial port open"

# Servo information
listOfServos = [0,1,2]										#All servo IDs
															
															#Leg Sections
															#Coxa - Femur - Tibia 
															#Array match this order
leg1Servos = [0,1,2]										#Leg 1 Servos
leg2Servos = [3,4,5]										#Leg 2 Servos
leg3Servos = [6,7,8]										#Leg 3 Servos
leg4Servos = [9,10,11]										#Leg 4 Servos
															#See: http://www.trossenrobotics.com/productdocs/assemblyguides/phantomx-hexapod.html
legSectionDefaults = [512,512,512]							#Default starting position per leg section
legLimitsCoxa = [240,790]									#Leg Section Servo low & high limit [LOW,HIGH]
legLimitsCoxa = [400,600]
legLimitsFemur = [240,790]									#Leg Section Servo low & high limit [LOW,HIGH]
#legLimitsFemur = [400,600]
legLimitsTibia = [400,600]									#Leg Section Servo low & high limit [LOW,HIGH]

bodyHeight = [8,6,2,14]										#Body height - Current,default, Min, Max

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
	data.append(0x0B) 										#Packet Bite Length
	data.append(0xFE) 										#Servo ID (pID) - 0xFE = ALL
	data.append(0x03) 										#CMD
	data.append(0x30) 										#Data - Command Address
	data.append(0x02) 										#Data - Command Length
	data.append(0x00) 										#Data - Write error = 0
	data.append(0x00) 										#Data - Write error = 0
	sendData(data);

# Torque On
def torqueOn():
	data = []
	data.append(0x0A) 										#Packet Bite Length
	data.append(0xFE) 										#Servo ID (pID) - 0xFE = ALL
	data.append(0x03) 										#CMD
	data.append(0x34) 										#Data - Command Address
	data.append(0x01) 										#Data - Command Length
	data.append(0x60) 										#Data - Torque On
	sendData(data);
	
# Set LED
def setLED(servoID,colorName):
	data = []
	data.append(0x0A) 										#Packet Bite Length
	data.append(servoID) 									#Servo ID (pID) - 0xFE = ALL
	data.append(0x03) 										#CMD
	data.append(0x35) 										#Data - Command Address
	data.append(0x01) 										#Data - Command Length
	data.append(colorName) 									#Data - LED Color 0x01=Green, 0x02 = Blue, 0x04 = Red
	sendData(data);
	
# Single Servo Move (I_JOG)
def singleServoMove(servoID,positionGoal,operatingTime,LEDcolorName):
	positionGoalMSB = int(positionGoal) >> 8
	positionGoalLSB = int(positionGoal) & 0xff
	data = []
	data.append(0x0C) 										#Packet Bite Length
	data.append(servoID) 									#Servo ID (pID) - 0xFE = ALL
	#data.append(0xFE) 										#Servo ID (pID) - 0xFE = ALL
	data.append(0x05) 										#CMD
	data.append(positionGoalLSB) 							#Data - JOG(LSB)
	data.append(positionGoalMSB) 							#Data - JOG(MSB)
	data.append(0x08) 										#Data - SET
	data.append(servoID) 									#Data - ID
	data.append(operatingTime) 								#Data - playtime
	sendData(data);	
	
# Send data to servo 
def sendData(data):	
	dataArrayLength=len(data)								# Get data Array Length
	ck1=checksum1(data,dataArrayLength) 					# Generate checksum 1
	ck2=checksum2(ck1)										# Generate checksum 2
	
	data.insert(0,0xFF)										# Add header
	data.insert(1,0xFF)										# Add header	
	data.insert(5,ck1)										# Insert checksum 1
	data.insert(6,ck2)										# Insert checksum 2
	stringToSend = ""
	for i in range(len(data)):
		byteFormat = '%02X' % data[i]
		stringToSend = stringToSend + "\\x" + byteFormat

	print stringToSend
	serPort.write(stringToSend.decode('string-escape'))

	print " "

# centerServos
def centerServos(listOfServos):
	for x in range(0, len(listOfServos)):
		singleServoMove(x,legSectionDefaults[x],60,x)

def ikCalc(bodyHeight,footDistance):	#ik Calc 
	#bodyHeight = 35
	#footDistance = 80
	femurLength = 70
	tibiaLength = 111
	hypo = math.sqrt(((math.pow(footDistance,2))+(math.pow(bodyHeight,2))))
	alpha = (math.acos((math.pow(femurLength,2)+math.pow(hypo,2)-math.pow(tibiaLength,2))/(2*femurLength*hypo)))*180/math.pi
	gamma = (math.atan2(bodyHeight,footDistance))*180/math.pi
	beta = 90-alpha+gamma
	beta = int(beta * 100) / 100.0
	delta = (math.acos((math.pow(femurLength,2)+math.pow(tibiaLength,2)-math.pow(hypo,2))/(2*femurLength*tibiaLength)))*180/math.pi
	epsilon = 180-delta
	epsilon = int(epsilon * 100) / 100.0
	print "Body Height: " + str(bodyHeight) + " Foot Distance: " + str(footDistance) + " Beta: " + str(beta) + " Epsilon: " + str(epsilon)
	return (beta,epsilon)

def gantMove():
	#Need to get direction and speed
	#Move 1 leg at a time using opposite legs (IE: 1,3,4,2)
	#while moving leg lean body into move with direction
	print "Gant move subsection"


def bodyTwist(twistSpeed,twistDirection):
	twistLimit = 10
	print "Body twist"
	
def testLoop():
	var = 1
	print "Loop start"
	while var == 1 :
		for x in range(0, 3):
			#legLimitsCoxa
			if x == 0:
				positionGoal = random.randint(legLimitsCoxa[0],legLimitsCoxa[1])
			if x == 1:
				positionGoal = random.randint(legLimitsFemur[0],legLimitsFemur[1])
			if x == 2:
				positionGoal = random.randint(legLimitsTibia[0],legLimitsTibia[1])
			#operatingTime = random.randint(1,254)
			operatingTime = 60
			singleServoMove(x,positionGoal,operatingTime,x)
			print "Servo: "
			print x
			print "Location: "
			print positionGoal
		time.sleep(10)
def testBodyMove():
	var = 1
	x = 0
	while var == 1 :
		if x == 100:
			x = 10
			legValues = ikCalc(x,80)
			#print "Femur: " + str(legValues[0])
			#femur
			if legValues[0] < 90:
				femurServo = ((90 - legValues[0])/.325) + 512
				print femurServo
			else:
				femurServo = ((90 + legValues[0])/.325)
				print femurServo
			#tibia
			if legValues[1] < 90:
				tibiaServo = ((90 - legValues[1])/.325) + 512
				print tibiaServo
			else:
				tibiaServo = ((90 + legValues[1])/.325) - 512
				print tibiaServo			
			#print "Tibia: " + str(legValues[1])
			operatingTime = 60
			singleServoMove(1,femurServo,operatingTime,x)
			singleServoMove(2,tibiaServo,operatingTime,x)
		else:
			x = 100
			legValues = ikCalc(x,80)
			#print "Femur: " + str(legValues[0])
			#femur
			if legValues[0] < 90:
				femurServo = ((90 - legValues[0])/.325) + 512
				print femurServo
			else:
				femurServo = ((90 + legValues[0])/.325)
				print femurServo
			#tibia
			if legValues[1] < 90:
				tibiaServo = ((90 - legValues[1])/.325) + 512
				print tibiaServo
			else:
				tibiaServo = ((90 + legValues[1])/.325) - 512
				print tibiaServo			
			#print "Tibia: " + str(legValues[1])
			operatingTime = 60
			singleServoMove(1,femurServo,operatingTime,x)
			singleServoMove(2,tibiaServo,operatingTime,x)
		time.sleep(2)
def start():
	print "Clearing errors"
	clearError()
	time.sleep(1)
	print "Turning torque on"
	torqueOn()
	time.sleep(1)
	print "Center servos"
	centerServos(leg1Servos)
	time.sleep(2)
	#testLoop()
	testBodyMove()


# Start of main application
start()