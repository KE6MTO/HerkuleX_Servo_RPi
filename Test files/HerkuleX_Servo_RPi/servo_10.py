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
from threading import Thread, Lock
from Queue import *
import pika

from datetime import datetime, timedelta

#print "Opening serial port"
serPort = serial.Serial('/dev/ttyAMA0', 115200, timeout=3.0)
print "Serial port open"

# Servo information
listOfServos = [0,1,2,3,4,5,6,7,8,9,10,11]					#All servo IDs
															
															#Leg Sections
															#Coxa - Femur - Tibia 
															#Array match this order
leg1Servos = [0,1,2]										#Leg 1 Servos - Front Right
leg2Servos = [3,4,5]										#Leg 2 Servos - Rear Right
leg3Servos = [6,7,8]										#Leg 3 Servos - Rear Left
leg4Servos = [9,10,11]										#Leg 4 Servos - Front Left
															#See: http://www.trossenrobotics.com/productdocs/assemblyguides/phantomx-hexapod.html

leg1ServoValues = [0,0,0]									#Leg 1 Servo Values
leg2ServoValues = [0,0,0]									#Leg 2 Servo Values
leg3ServoValues = [0,0,0]									#Leg 3 Servo Values
leg4ServoValues = [0,0,0]									#Leg 4 Servo Values
															
legLimitsCoxa = [400,600]
legLimitsFemur = [400,600]									#Leg Section Servo low & high limit [LOW,HIGH]
legLimitsTibia = [400,600]									#Leg Section Servo low & high limit [LOW,HIGH]

bodyHeightLimits = [2,14]									#Body height limits - Min, Max
bodyHeightCurrent = [8,8,8,8]								#Body height current - each leg, start with default 
rabbitMQHost = 'localhost'

legSectionDefaults = [512,512,512]							#Coxa - Femur - Tibia defaults

LENGTH_COXA = 60.0
LENGTH_FEMUR = 70.0
LENGTH_TIBIA = 60.0

# checksum1
def checksum1(data, lenghtString):
	buffer = 0	
	for x in range(0, lenghtString):
		buffer = buffer ^ data[x]
	return buffer&0xFE;

# checksum2
def checksum2(XOR):
	return (~XOR)&0xFE

# ServoStatus
#
#        self.exceed_input_voltage_limit = (reg48 & 0x01) != 0
#        self.exceed_allowed_pot_limit = (reg48 & 0x02) != 0
#        self.exceed_temperature_limit = (reg48 & 0x04) != 0
#        self.invalid_packet = (reg48 & 0x08) != 0
#        self.overload_detected = (reg48 & 0x10) != 0
#        self.driver_fault_detected = (reg48 & 0x20) != 0
#        self.eep_reg_distorted = (reg48 & 0x40) != 0
#
#        self.moving = (reg49 & 0x01) != 0
#        self.inposition = (reg49 & 0x02) != 0
#        self.checksum_error = (reg49 & 0x04) != 0
#        self.unknown_command = (reg49 & 0x08) != 0
#        self.exceed_reg_range = (reg49 & 0x10) != 0
#        self.garbage_detected = (reg49 & 0x20) != 0
#        self.motor_on = (reg49 & 0x40) != 0	

'''
    def status(self, servo):
        received = yield From(self.send_recv_packet(servo, self.CMD_STAT, ''))
        assert received.servo == servo or servo == self.BROADCAST
        assert received.cmd == (0x40 | self.CMD_STAT)
        assert len(received.data) == 2

        reg48 = ord(received.data[0])
        reg49 = ord(received.data[1])

        result = StatusResponse(reg48, reg49)

        raise Return(result)
'''
		
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
	if LEDcolorName == "GREEN":
		data.append(0x4) 	
	elif LEDcolorName == "BLUE":
		data.append(0x8) 
	elif LEDcolorName == "RED":
		data.append(0x10) 
	else:
		data.append(0x8) 										#Data - SET
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

	#print stringToSend
	serPort.write(stringToSend.decode('string-escape'))

	#print " "

# centerServos
def centerServos():

	#legServoValues = servoCal2(10,40)
	legServoValues = ikLeg(50,110)
	#Leg 1
	#Center Coxa
	singleServoMove(leg1Servos[0],legSectionDefaults[0],60,"BLUE")
	#singleServoMove(leg1Servos[1],legSectionDefaults[1],60,"BLUE")
	#singleServoMove(leg1Servos[2],legSectionDefaults[2],60,"BLUE")
	singleServoMove(leg1Servos[1],legServoValues[0],60,"GREEN")
	singleServoMove(leg1Servos[2],legServoValues[1],60,"GREEN")
	
	#Leg 2
	#Center Coxa
	singleServoMove(leg2Servos[0],legSectionDefaults[0],60,"BLUE")
	#singleServoMove(leg2Servos[1],legSectionDefaults[1],60,"BLUE")
	#singleServoMove(leg2Servos[2],legSectionDefaults[2],60,"BLUE")	
	singleServoMove(leg2Servos[1],legServoValues[0],60,"GREEN")
	singleServoMove(leg2Servos[2],legServoValues[1],60,"GREEN")

	#Leg 3
	#Center Coxa
	singleServoMove(leg3Servos[0],legSectionDefaults[0],60,"BLUE")
	#singleServoMove(leg3Servos[1],legSectionDefaults[1],60,"BLUE")
	#singleServoMove(leg3Servos[2],legSectionDefaults[2],60,"BLUE")
	singleServoMove(leg3Servos[1],legServoValues[0],60,"GREEN")
	singleServoMove(leg3Servos[2],legServoValues[1],60,"GREEN")
	
	#Leg 4
	#Center Coxa
	singleServoMove(leg4Servos[0],legSectionDefaults[0],60,"BLUE")
	#singleServoMove(leg4Servos[1],legSectionDefaults[1],60,"BLUE")
	#singleServoMove(leg4Servos[2],legSectionDefaults[2],60,"BLUE")
	singleServoMove(leg4Servos[1],legServoValues[0],60,"GREEN")
	singleServoMove(leg4Servos[2],legServoValues[1],60,"GREEN")
	
	#for x in range(0, len(listOfServos)):
	#	singleServoMove(x,legSectionDefaults[x],60,x)



def ikLeg(x, y):
	print "IK function called. x=", x, "y=", y,
	print " "
	try:
		d = math.sqrt(x*x+y*y)
		k = (d*d-LENGTH_TIBIA*LENGTH_TIBIA+LENGTH_FEMUR*LENGTH_FEMUR)/(2*d)
		m = math.sqrt(LENGTH_FEMUR*LENGTH_FEMUR-k*k)
	except ZeroDivisionError:
		print "Divide by Zero error. No valid joint solution."
		return "fail"
	except ValueError:
		print "Math function error. Probably square root of negative number. No valid joint solution."
		return "fail"
	theta = math.degrees(math.atan2(float(y),float(x))-math.atan2(m,k))
	phi   = math.degrees(math.atan2(m,k)+math.atan2(m,(d-k)))
	#returnAngles = [theta, -phi]
	print "Femur Angle=", theta, "Tibia Angle=", phi
	
	if theta >= 0:
		femurServoValue = 512+((theta)/0.325)
	else:
		femurServoValue = 512-((theta*-1)/0.325)

	if phi >= 0:
		tibiaServoValue = 512-((phi)/0.325)
	else:
		tibiaServoValue = 512+((phi*-1)/0.325)

	return (femurServoValue,tibiaServoValue)

def calcLegValues(step_low,step_high,start_x,start_y,end_x,end_y):
	debug = "true"
	if debug == "true": #show debug information
		print " "
		print "Leg Calc Function"
		print " "
	
	#End low calc
	m = (start_y-0)/float((start_x-0))
	targetAngle=(math.atan(m)*180)/math.pi
	updated_base_x_coord= 0 + (LENGTH_COXA * math.cos((targetAngle*math.pi)/180))
	updated_base_y_coord= 0 + (LENGTH_COXA* math.sin((targetAngle*math.pi)/180))	
	targetDistance=math.sqrt(math.pow(start_x-updated_base_x_coord, 2)+math.pow(start_y-updated_base_y_coord, 2))	
	adjustedTargetAngle=targetAngle-45
	#startCoxaServo=512+(adjustedTargetAngle/.325) #Leg Coxa
	startCoxaServo=adjustedTargetAngle #Leg Coxa
	if debug == "true": #show debug information
		print "  Leg end low calc"
		print "  M Value: " + str(m)
		print "  Coxa Angle: " + str(targetAngle)
		print "  Femur X Coord: " + str(updated_base_x_coord)
		print "  Femur Y Coord: " + str(updated_base_y_coord)
		print "  Tibia distance: " + str(targetDistance)
		print "  Coxa Servo Value: " + str(startCoxaServo)
	startLegServoValues = ikLeg(targetDistance,step_low)

	step_end_low_coxa_servo_value = targetAngle
	step_end_low_femur_Servo_value = startLegServoValues[0]
	step_end_low_tibia_Servo_value = startLegServoValues[1]
	
	#End high calc
	m = (start_y-0)/float((start_x-0))
	targetAngle=(math.atan(m)*180)/math.pi
	updated_base_x_coord= 0 + (LENGTH_COXA * math.cos((targetAngle*math.pi)/180))
	updated_base_y_coord= 0 + (LENGTH_COXA* math.sin((targetAngle*math.pi)/180))	
	targetDistance=math.sqrt(math.pow(start_x-updated_base_x_coord, 2)+math.pow(start_y-updated_base_y_coord, 2))	
	adjustedTargetAngle=targetAngle-45
	startCoxaServo=adjustedTargetAngle #Leg Coxa
	if debug == "true": #show debug information
		print "  Leg end high calc"
		print "  M Value: " + str(m)
		print "  Coxa Angle: " + str(targetAngle)
		print "  Femur X Coord: " + str(updated_base_x_coord)
		print "  Femur Y Coord: " + str(updated_base_y_coord)
		print "  Tibia distance: " + str(targetDistance)
		print "  Coxa Servo Value: " + str(startCoxaServo)
	startLegServoValues = ikLeg(targetDistance,step_high)	
	
	step_end_high_coxa_servo_value = targetAngle
	step_end_high_femur_Servo_value = startLegServoValues[0]
	step_end_high_tibia_Servo_value = startLegServoValues[1]
	
	#Start calc
	m = (end_y-0)/float((end_x-0))
	targetAngle=(math.atan(m)*180)/math.pi
	updated_base_x_coord= 0 + (LENGTH_COXA * math.cos((targetAngle*math.pi)/180))
	updated_base_y_coord= 0 + (LENGTH_COXA* math.sin((targetAngle*math.pi)/180))	
	targetDistance=math.sqrt(math.pow(end_x-updated_base_x_coord, 2)+math.pow(end_y-updated_base_y_coord, 2))	
	adjustedTargetAngle=targetAngle-45
	startCoxaServo=adjustedTargetAngle #Leg Coxa
	if debug == "true": #show debug information
		print "  Leg start high calc"
		print "  M Value: " + str(m)
		print "  Coxa Angle: " + str(targetAngle)
		print "  Femur X Coord: " + str(updated_base_x_coord)
		print "  Femur Y Coord: " + str(updated_base_y_coord)
		print "  Tibia distance: " + str(targetDistance)
		print "  Coxa Servo Value: " + str(startCoxaServo)
	startLegServoValues = ikLeg(targetDistance,step_low)		

	step_start_coxa_servo_value = targetAngle
	step_start_femur_Servo_value = startLegServoValues[0]
	step_start_tibia_Servo_value = startLegServoValues[1]
	return (step_end_low_coxa_servo_value,step_end_low_femur_Servo_value,step_end_low_tibia_Servo_value,step_end_high_coxa_servo_value,step_end_high_femur_Servo_value,step_end_high_tibia_Servo_value,step_start_coxa_servo_value,step_start_femur_Servo_value,step_start_tibia_Servo_value) 


def	walkTest():
	legValues = calcLegValues(110,80,80,80,80,-10) #step_low,step_high,start_x,start_y,end_x,end_y
	print legValues[0]
	print legValues[1]
	print legValues[2]
	print legValues[3]
	print legValues[4]
	print legValues[5]
	print legValues[6]
	print legValues[7]
	print legValues[8]
	while True :
		speed = 60
		sleeptime = speed/100.0
		step_sysle_time=60/4.0/100.0
		#print sleeptime
		time.sleep(sleeptime)
		print "start"
		singleServoMove(leg1Servos[0],512-(legValues[0]/.325),speed,"RED")#512+(legValues[0]/.325)
		singleServoMove(leg1Servos[1],legValues[1],speed,"GREEN")
		singleServoMove(leg1Servos[2],legValues[2],speed,"GREEN")
		
		singleServoMove(leg2Servos[0],512+(legValues[0]/.325),speed,"BLUE")#512+(legValues[0]/.325)
		singleServoMove(leg2Servos[1],legValues[1],speed,"GREEN")
		singleServoMove(leg2Servos[2],legValues[2],speed,"GREEN")

		singleServoMove(leg3Servos[0],512-(legValues[0]/.325),speed,"BLUE")#512+(legValues[0]/.325)
		singleServoMove(leg3Servos[1],legValues[1],speed,"GREEN")
		singleServoMove(leg3Servos[2],legValues[2],speed,"GREEN")

		singleServoMove(leg4Servos[0],512+(legValues[0]/.325),speed,"BLUE")#512+(legValues[0]/.325)
		singleServoMove(leg4Servos[1],legValues[1],speed,"GREEN")
		singleServoMove(leg4Servos[2],legValues[2],speed,"GREEN")		
		time.sleep(sleeptime)
		print "end"
		singleServoMove(leg1Servos[0],512-(legValues[3]/.325),speed,"RED")#512+(legValues[0]/.325)
		singleServoMove(leg1Servos[1],legValues[4],speed,"GREEN")
		singleServoMove(leg1Servos[2],legValues[5],speed,"GREEN")
		
		singleServoMove(leg2Servos[0],512+(legValues[3]/.325),speed,"BLUE")#512+(legValues[0]/.325)
		singleServoMove(leg2Servos[1],legValues[4],speed,"GREEN")
		singleServoMove(leg2Servos[2],legValues[5],speed,"GREEN")

		singleServoMove(leg3Servos[0],512-(legValues[3]/.325),speed,"BLUE")#512+(legValues[0]/.325)
		singleServoMove(leg3Servos[1],legValues[4],speed,"GREEN")
		singleServoMove(leg3Servos[2],legValues[5],speed,"GREEN")
		
		singleServoMove(leg4Servos[0],512+(legValues[3]/.325),speed,"BLUE")#512+(legValues[0]/.325)
		singleServoMove(leg4Servos[1],legValues[4],speed,"GREEN")
		singleServoMove(leg4Servos[2],legValues[5],speed,"GREEN")
		time.sleep(sleeptime)
		print "end"

		singleServoMove(leg1Servos[0],512-(legValues[6]/.325),speed,"RED")#512+(legValues[0]/.325)
		singleServoMove(leg1Servos[1],legValues[7],speed,"GREEN")
		singleServoMove(leg1Servos[2],legValues[8],speed,"GREEN")
		
		singleServoMove(leg2Servos[0],512+(legValues[6]/.325),speed,"BLUE")#512+(legValues[0]/.325)
		singleServoMove(leg2Servos[1],legValues[7],speed,"GREEN")
		singleServoMove(leg2Servos[2],legValues[8],speed,"GREEN")

		singleServoMove(leg3Servos[0],512-(legValues[6]/.325),speed,"BLUE")#512+(legValues[0]/.325)
		singleServoMove(leg3Servos[1],legValues[7],speed,"GREEN")
		singleServoMove(leg3Servos[2],legValues[8],speed,"GREEN")
		
		singleServoMove(leg4Servos[0],512+(legValues[6]/.325),speed,"BLUE")#512+(legValues[0]/.325)
		singleServoMove(leg4Servos[1],legValues[7],speed,"GREEN")
		singleServoMove(leg4Servos[2],legValues[8],speed,"GREEN")

	
def start():
	print "Clearing errors"
	clearError()
	time.sleep(1)
	print "Turning torque on"
	torqueOn()
	time.sleep(1)
	print "Center servos"
	centerServos()
	time.sleep(1)
	walkTest()





# Start of main application
start()
