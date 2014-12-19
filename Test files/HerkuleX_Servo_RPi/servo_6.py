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
#serPort = serial.Serial('/dev/ttyAMA0', 115200, timeout=3.0)
serPort = serial.Serial('/dev/ttyUSB0', 115200, timeout=3.0)
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

# Commands
EEP_WRITE_REQ = 0x01
EEP_READ_REQ  = 0x02
RAM_WRITE_REQ = 0x03
RAM_READ_REQ  = 0x04
I_JOG_REQ     = 0x05
S_JOG_REQ     = 0x06
STAT_REQ      = 0x07
ROLLBACK_REQ  = 0x08
REBOOT_REQ    = 0x09

EEP_WRITE_ACK = 0x41
EEP_READ_ACK  = 0x42
RAM_WRITE_ACK = 0x43
RAM_READ_ACK  = 0x44
I_JOG_ACK     = 0x45
S_JOG_ACK     = 0x46
STAT_ACK      = 0x47
ROLLBACK_ACK  = 0x48
REBOOT_ACK    = 0x49


#Addresses
MODEL_NO1_EEP                        = 0
MODEL_NO2_EEP                        = 1
VERSION1_EEP                         = 2
VERSION2_EEP                         = 3
BAUD_RATE_EEP                        = 4
SERVO_ID_EEP                         = 6
SERVO_ID_RAM                         = 0
ACK_POLICY_EEP                       = 7
ACK_POLICY_RAM                       = 1
ALARM_LED_POLICY_EEP                 = 8
ALARM_LED_POLICY_RAM                 = 2
TORQUE_POLICY_EEP                    = 9
TORQUE_POLICY_RAM                    = 3
MAX_TEMP_EEP                         = 11
MAX_TEMP_RAM                         = 5
MIN_VOLTAGE_EEP                      = 12
MIN_VOLTAGE_RAM                      = 6
MAX_VOLTAGE_EEP                      = 13
MAX_VOLTAGE_RAM                      = 7
ACCELERATION_RATIO_EEP               = 14
ACCELERATION_RATIO_RAM               = 8
MAX_ACCELERATION_TIME_EEP            = 15
MAX_ACCELERATION_TIME_RAM            = 9
DEAD_ZONE_EEP                        = 16
DEAD_ZONE_RAM                        = 10
SATURATOR_OFFSET_EEP                 = 17 
SATURATOR_OFFSET_RAM                 = 11
SATURATOR_SLOPE_EEP                  = 18
SATURATOR_SLOPE_RAM                  = 12
PWM_OFFSET_EEP                       = 20
PWM_OFFSET_RAM                       = 14
MIN_PWM_EEP                          = 21
MIN_PWM_RAM                          = 15
MAX_PWM_EEP                          = 22
MAX_PWM_RAM                          = 16
OVERLOAD_PWM_THRESHOLD_EEP           = 24
OVERLOAD_PWM_THRESHOLD_RAM           = 18
MIN_POSITION_EEP                     = 26
MIN_POSITION_RAM                     = 20
MAX_POSITION_EEP                     = 28
MAX_POSITION_RAM                     = 22
POSITION_KP_EEP                      = 30
POSITION_KP_RAM                      = 24
POSITION_KD_EEP                      = 32
POSITION_KD_RAM                      = 26
POSITION_KI_EEP                      = 34
POSITION_KI_RAM                      = 28
POSITION_FEEDFORWARD_GAIN1_EEP       = 36
POSITION_FEEDFORWARD_GAIN1_RAM       = 30
POSITION_FEEDFORWARD_GAIN2_EEP       = 38
POSITION_FEEDFORWARD_GAIN2_RAM       = 32
VELOCITY_KP_EEP                      = 40
VELOCITY_KP_RAM                      = 34
VELOCITY_KI_EEP                      = 42
VELOCITY_KI_RAM                      = 36
LED_BLINK_PERIOD_EEP                 = 44
LED_BLINK_PERIOD_RAM                 = 38
ADC_FAULT_CHECK_PERIOD_EEP           = 45
ADC_FAULT_CHECK_PERIOD_RAM           = 39
PACKET_GARBAGE_CHECK_PERIOD_EEP      = 46
PACKET_GARBAGE_CHECK_PERIOD_RAM      = 40
STOP_DETECTION_PERIOD_EEP            = 47
STOP_DETECTION_PERIOD_RAM            = 41
OVERLOAD_DETECTION_PERIOD_EEP        = 48
OVERLOAD_DETECTION_PERIOD_RAM        = 42
STOP_THRESHOLD_EEP                   = 49
STOP_THRESHOLD_RAM                   = 43
INPOSITION_MARGIN_EEP                = 50
INPOSITION_MARGIN_RAM                = 44
CALIBRATION_DIFF_LOW_EEP             = 52
CALIBRATION_DIFF_LOW_RAM             = 46
CALIBRATION_DIFF_UP_EEP              = 53
CALIBRATION_DIFF_UP_RAM              = 47
STATUS_ERROR_RAM                     = 48
STATUS_DETAIL_RAM                    = 49
AUX1_RAM                             = 50
TORQUE_CONTROL_RAM                   = 52
LED_CONTROL_RAM                      = 53
VOLTAGE_RAM                          = 54
TEMPERATURE_RAM                      = 55
CURRENT_CONTROL_MODE_RAM             = 56
TICK_RAM                             = 57
CALIBRATED_POSITION_RAM              = 58
ABSOLUTE_POSITION_RAM                = 60
DIFFERENTIAL_POSITION_RAM            = 62
PWM_RAM                              = 64
ABSOLUTE_SECOND_POSITION_RAM         = 66
ABSOLUTE_GOAL_POSITION_RAM           = 68
ABSOLUTE_DESIRED_TRAJECTORY_POSITION = 70
DESIRED_VELOCITY_RAM                 = 72

BYTE1 = 0x01
BYTE2 = 0x02

BROADCAST_ID = 0xFE

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

	print stringToSend
	serPort.write(stringToSend.decode('string-escape'))

	#print " "

# centerServos
def centerServos():
	#legServoValues = servoCal(1,63,40)
	legServoValues = servoCal2(10,40)
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

def ikCalc(bodyHeight,footDistance):	#ik Calc 
	#bodyHeight = 35
	#footDistance = 80
	femurLength = 70
	tibiaLength = 60 #was 111
	#footDistance = footDistance + 60# Add 60 for distance from body
	hypo = math.sqrt(((math.pow(footDistance,2))+(math.pow(bodyHeight,2))))
	alpha = (math.acos((math.pow(femurLength,2)+math.pow(hypo,2)-math.pow(tibiaLength,2))/(2*femurLength*hypo)))*180/math.pi
	gamma = (math.atan2(bodyHeight,footDistance))*180/math.pi
	beta = 90-alpha+gamma
	beta = int(beta * 100) / 100.0
	delta = (math.acos((math.pow(femurLength,2)+math.pow(tibiaLength,2)-math.pow(hypo,2))/(2*femurLength*tibiaLength)))*180/math.pi
	epsilon = 180-delta
	epsilon = int(epsilon * 100) / 100.0
	#print "Body Height: " + str(bodyHeight) + " Foot Distance: " + str(footDistance) + " Beta: " + str(beta) + " Epsilon: " + str(epsilon)
	return (beta,epsilon)

def servoGetDataTest():
	print "Get servo data"
	data = []
	data.append(0x09)	#Packet Bite Length
	data.append(0x01)	#Servo ID (pID) - 0xFE = ALL
	data.append(0x04)	#CMD
	data.append(0x35)	#Data 1
	data.append(0x01)	#Data 2
	sendData(data);	
	rxdata = []
	#rxdata = serPort.read()
	rxdata = serPort.read(12)
	#print rxdata
	print rxdata[0]
	print rxdata[1]
	print rxdata[2]
	print rxdata[3]
	print rxdata[4]
	print rxdata[5]
	print rxdata[6]
	print rxdata[7]
	print rxdata[8]
	print rxdata[9]
	print rxdata[10]
	print rxdata[11]
	
	print len(rxdata)
	print ord(rxdata[9])&0xFF
	print ord(rxdata[11])&0xFF

def servoCal(leg,bodyHeight,footDistance):
	legValues = ikCalc(bodyHeight,footDistance)
	if legValues[0] <= 90:
		#print "betaServo 1 - servoCal <= 90"
		betaServo = 512-((90-legValues[0])/0.325)
		#femurServo = ((90 - legValues[0])/.325) + 512
	else:
		#femurServo = ((90 + legValues[0])/.325)
		betaServo = 512+((legValues[0]-90)/0.325)
	if legValues[1] <= 90:
		#print "Epsilon 2 - servoCal <= 90"
		epsilonServo = 512-((90-legValues[1])/0.325)
		#femurServo = ((90 - legValues[0])/.325) + 512
	else:
		#print "Epsilon 2 - servoCal > 90"
		#femurServo = ((90 + legValues[0])/.325)
		epsilonServo = 512+((legValues[1]-90)/0.325)
	#print "Beta Servo: " + str(betaServo) + "  Epsilon Servo: " + str(epsilonServo)
	return (betaServo,epsilonServo)

def servoCal2(bodyHeight,footDistance):
	legValues = ikCalc(bodyHeight,footDistance)
	if legValues[0] <= 90:
		#print "betaServo 1 - servoCal <= 90"
		betaServo = 512-((90-legValues[0])/0.325)
		#femurServo = ((90 - legValues[0])/.325) + 512
		print "BetaServer <= 90"
	else:
		#femurServo = ((90 + legValues[0])/.325)
		betaServo = 512+((legValues[0]-90)/0.325)
		print "BetaServer > 90"
	if legValues[1] <= 90:
		#print "Epsilon 2 - servoCal <= 90"
		epsilonServo = 512-((90-legValues[1])/0.325)
		#femurServo = ((90 - legValues[0])/.325) + 512
		print "epsilonServo <= 90"
	else:
		#print "Epsilon 2 - servoCal > 90"
		#femurServo = ((90 + legValues[0])/.325)
		epsilonServo = 512+((legValues[1]-90)/0.325)
		print "epsilonServo > 90"
	#print "Beta Servo: " + str(betaServo) + "  Epsilon Servo: " + str(epsilonServo)
	return (betaServo,epsilonServo)



def callback(ch, method, properties, body):
	ticks = time.time()
	commandSendTime = float(body.split(',')[2])
	if ticks >= (commandSendTime + 5):
		print "commandData too old"
		packet_to_send="$cmdDataErr," + "commandData too old" + "," + str(ticks) + ",$" 
		channel.basic_publish(exchange='',routing_key='telemetryData',body=packet_to_send)
	else:
		commandDataSent = body.split(',')[1]
		print "Received Command: " + commandDataSent
		queue.put(commandDataSent)
		
		# print " [x] Received %r" % (body,)
	
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
	#setLED(0xFE,0x04)
	time.sleep(1)
	#testLoop()
	#manualTest()
	#testWalk()
	servoGetDataTest()
	#userInputTest()
	#testBodyMove()
	#testLegMove()


# Start of main application
start()
