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

def gantMove():
	#Need to get direction and speed
	#Move 1 leg at a time using opposite legs (IE: 1,3,4,2)
	#while moving leg lean body into move with direction
	print "Gant move subsection"


def userInputTest():
	while True :
		#userLegDis=0
		#userbodyHigh=0
		#legServoValues[0]=512
		#legServoValues[1]=512
		userLegDis = input('Leg Distance (40): ')
		userbodyHigh = input('Body Height (63): ')
		userLegDis = int(userLegDis)
		userbodyHigh = int(userbodyHigh)
		legServoValues = servoCal2(userbodyHigh,userLegDis) #bodyHeight,footDistance
		print legServoValues[0]
		print legServoValues[1]
		singleServoMove(1,legServoValues[0],25,"GREEN")
		singleServoMove(2,legServoValues[1],25,"GREEN")
		singleServoMove(4,legServoValues[0],25,"GREEN")
		singleServoMove(5,legServoValues[1],25,"GREEN")
		singleServoMove(7,legServoValues[0],25,"GREEN")
		singleServoMove(8,legServoValues[1],25,"GREEN")
		singleServoMove(10,legServoValues[0],25,"GREEN")
		singleServoMove(11,legServoValues[1],25,"GREEN")


def testWalk():
	print "Start Walking Test"
	operatingTime = 50
	pauseTime = .9
	pauseTime2 = .9
	legToSwing=1
	timeTarget = datetime.now() + timedelta(seconds=3)
	while True:
		if datetime.now() >= timeTarget:
			print "Swinging leg #" + str(int(legToSwing))
			if legToSwing == 1:
				legServoValues = servoCal2(15,15) #bodyHeight,footDistance
				#was 70,30
				singleServoMove(1,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(2,legServoValues[1],operatingTime/2,"GREEN")
				#time.sleep(.1)
				singleServoMove(0,650,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(6,558,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(3,466,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(9,374,operatingTime,"RED") #Move Coxa Servo
				legServoValues = servoCal2(15,40)
				time.sleep(pauseTime)

				singleServoMove(1,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(2,legServoValues[1],operatingTime/2,"GREEN")
				time.sleep(pauseTime2)
				legToSwing = 4
			if legToSwing == 2:
				legServoValues = servoCal2(15,15) #bodyHeight,footDistance
				#was 70,30
				singleServoMove(4,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(5,legServoValues[1],operatingTime/2,"GREEN")
				#time.sleep(.1)
				singleServoMove(3,650,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(9,558,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(0,466,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(6,374,operatingTime,"RED") #Move Coxa Servo
				legServoValues = servoCal2(15,40)
				time.sleep(pauseTime)
				singleServoMove(4,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(5,legServoValues[1],operatingTime/2,"GREEN")
				time.sleep(pauseTime2)
				legToSwing = 3
			if legToSwing == 3:
				legServoValues = servoCal2(15,15) #bodyHeight,footDistance
				#was 70,30
				singleServoMove(7,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(8,legServoValues[1],operatingTime/2,"GREEN")
				#time.sleep(.1)
				singleServoMove(6,650,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(3,558,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(9,466,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(0,374,operatingTime,"RED") #Move Coxa Servo
				legServoValues = servoCal2(15,40)
				time.sleep(pauseTime)
				singleServoMove(7,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(8,legServoValues[1],operatingTime/2,"GREEN")
				time.sleep(pauseTime2)
				legToSwing = 1
			if legToSwing == 4:
				legServoValues = servoCal2(15,15) #bodyHeight,footDistance
				#was 70,30
				singleServoMove(10,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(11,legServoValues[1],operatingTime/2,"GREEN")
				#time.sleep(.1)
				singleServoMove(9,650,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(0,558,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(6,466,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(3,374,operatingTime,"RED") #Move Coxa Servo
				legServoValues = servoCal2(15,40)
				time.sleep(pauseTime)
				singleServoMove(10,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(11,legServoValues[1],operatingTime/2,"GREEN")
				time.sleep(pauseTime2)
				legToSwing = 2

			#timeTarget = datetime.now() + timedelta(seconds=1)		
		#time.sleep(.1)

def testLegSwing():
	#Starting with leg contacting surface
	#Start foot lift (Femur & Tibia servos)
	#Lift by "lowering" the body value
	#Start leg swing (Coxa servo)
	#Start foot lower
	operatingTime = 50
	print "Start Leg Swing Test"
	#timeCurrent = datetime.now()
	timeTarget = datetime.now() + timedelta(seconds=3)
	moveTotalTime = 200
	moveSpaceTime = 3
	while True :
		coord_A2=15
		#was 28
		coord_Y2=15
		coord_A1=0
		coord_Y1=5
		coxa_length=5
		for x in range(0, 29):
			print "  "
			coord_Y2 = x
			print "We're on step %d" % (x)			
			#targetDistance=math.sqrt(math.pow(coord_A2-coord_A1, 2)+math.pow(coord_Y2-coord_Y1, 2))
			#print targetDistance
			m = (coord_Y2-coord_Y1)/float((coord_A2-coord_A1))
			#print m
			targetAngle=(math.atan(m)*180)/math.pi
			#print targetAngle
			#(targetAngle*math.pi)/180
			updated_base_x_coord=(coord_A1 + coxa_length) * math.cos((targetAngle*math.pi)/180)
			#=B14 + 5 * SIN(G14)
			updated_base_y_coord=(coord_Y1) * math.sin((targetAngle*math.pi)/180)
			
			targetDistance=math.sqrt(math.pow(coord_A2-updated_base_x_coord, 2)+math.pow(coord_Y2-updated_base_y_coord, 2))
			print targetDistance
			
			legServoValues = servoCal2(targetDistance,40) #bodyHeight,footDistance
			print legServoValues
			singleServoMove(1,legServoValues[0],20,"GREEN")
			singleServoMove(2,legServoValues[1],20,"GREEN")
			CoxaServo=650.46-(targetAngle/.325)
			singleServoMove(0,CoxaServo,20,"RED") #Move Coxa Servo
			#legServoValues = servoCal2(15,15) #bodyHeight,footDistance
			#singleServoMove(1,legServoValues[0],operatingTime/2,"GREEN")
			#singleServoMove(2,legServoValues[1],operatingTime/2,"GREEN")
			#singleServoMove(0,650,operatingTime,"RED") #Move Coxa Servo
			time.sleep(.5)

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
	testLegSwing()
	#userInputTest()
	#testBodyMove()
	#testLegMove()


# Start of main application
queue = Queue()
start()
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

def gantMove():
	#Need to get direction and speed
	#Move 1 leg at a time using opposite legs (IE: 1,3,4,2)
	#while moving leg lean body into move with direction
	print "Gant move subsection"


def userInputTest():
	while True :
		#userLegDis=0
		#userbodyHigh=0
		#legServoValues[0]=512
		#legServoValues[1]=512
		userLegDis = input('Leg Distance (40): ')
		userbodyHigh = input('Body Height (63): ')
		userLegDis = int(userLegDis)
		userbodyHigh = int(userbodyHigh)
		legServoValues = servoCal2(userbodyHigh,userLegDis) #bodyHeight,footDistance
		print legServoValues[0]
		print legServoValues[1]
		singleServoMove(1,legServoValues[0],25,"GREEN")
		singleServoMove(2,legServoValues[1],25,"GREEN")
		singleServoMove(4,legServoValues[0],25,"GREEN")
		singleServoMove(5,legServoValues[1],25,"GREEN")
		singleServoMove(7,legServoValues[0],25,"GREEN")
		singleServoMove(8,legServoValues[1],25,"GREEN")
		singleServoMove(10,legServoValues[0],25,"GREEN")
		singleServoMove(11,legServoValues[1],25,"GREEN")


def testWalk():
	print "Start Walking Test"
	operatingTime = 50
	pauseTime = .9
	pauseTime2 = .9
	legToSwing=1
	timeTarget = datetime.now() + timedelta(seconds=3)
	while True:
		if datetime.now() >= timeTarget:
			print "Swinging leg #" + str(int(legToSwing))
			if legToSwing == 1:
				legServoValues = servoCal2(15,15) #bodyHeight,footDistance
				#was 70,30
				singleServoMove(1,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(2,legServoValues[1],operatingTime/2,"GREEN")
				#time.sleep(.1)
				singleServoMove(0,650,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(6,558,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(3,466,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(9,374,operatingTime,"RED") #Move Coxa Servo
				legServoValues = servoCal2(15,40)
				time.sleep(pauseTime)

				singleServoMove(1,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(2,legServoValues[1],operatingTime/2,"GREEN")
				time.sleep(pauseTime2)
				legToSwing = 4
			if legToSwing == 2:
				legServoValues = servoCal2(15,15) #bodyHeight,footDistance
				#was 70,30
				singleServoMove(4,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(5,legServoValues[1],operatingTime/2,"GREEN")
				#time.sleep(.1)
				singleServoMove(3,650,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(9,558,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(0,466,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(6,374,operatingTime,"RED") #Move Coxa Servo
				legServoValues = servoCal2(15,40)
				time.sleep(pauseTime)
				singleServoMove(4,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(5,legServoValues[1],operatingTime/2,"GREEN")
				time.sleep(pauseTime2)
				legToSwing = 3
			if legToSwing == 3:
				legServoValues = servoCal2(15,15) #bodyHeight,footDistance
				#was 70,30
				singleServoMove(7,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(8,legServoValues[1],operatingTime/2,"GREEN")
				#time.sleep(.1)
				singleServoMove(6,650,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(3,558,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(9,466,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(0,374,operatingTime,"RED") #Move Coxa Servo
				legServoValues = servoCal2(15,40)
				time.sleep(pauseTime)
				singleServoMove(7,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(8,legServoValues[1],operatingTime/2,"GREEN")
				time.sleep(pauseTime2)
				legToSwing = 1
			if legToSwing == 4:
				legServoValues = servoCal2(15,15) #bodyHeight,footDistance
				#was 70,30
				singleServoMove(10,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(11,legServoValues[1],operatingTime/2,"GREEN")
				#time.sleep(.1)
				singleServoMove(9,650,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(0,558,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(6,466,operatingTime,"RED") #Move Coxa Servo
				singleServoMove(3,374,operatingTime,"RED") #Move Coxa Servo
				legServoValues = servoCal2(15,40)
				time.sleep(pauseTime)
				singleServoMove(10,legServoValues[0],operatingTime/2,"GREEN")
				singleServoMove(11,legServoValues[1],operatingTime/2,"GREEN")
				time.sleep(pauseTime2)
				legToSwing = 2

			#timeTarget = datetime.now() + timedelta(seconds=1)		
		#time.sleep(.1)

def testLegSwing():
	#Starting with leg contacting surface
	#Start foot lift (Femur & Tibia servos)
	#Lift by "lowering" the body value
	#Start leg swing (Coxa servo)
	#Start foot lower
	operatingTime = 50
	print "Start Leg Swing Test"
	#timeCurrent = datetime.now()
	timeTarget = datetime.now() + timedelta(seconds=3)
	moveTotalTime = 200
	moveSpaceTime = 3
	while True :
		time.sleep(50)
		coord_A2=15
		#was 28
		coord_Y2=15
		coord_A1=0
		coord_Y1=0
		coxa_length=5
		for x in range(0, 29):
			time.sleep(50)
			print "  "
			coord_Y2 = x
			print "We're on step %d" % (x)			
			#targetDistance=math.sqrt(math.pow(coord_A2-coord_A1, 2)+math.pow(coord_Y2-coord_Y1, 2))
			#print targetDistance
			m = (coord_Y2-coord_Y1)/float((coord_A2-coord_A1))
			print m
			targetAngle=(math.atan(m)*180)/math.pi
			print targetAngle
			#(targetAngle*math.pi)/180
			updated_base_x_coord=(coord_A1 + coxa_length) * math.cos((targetAngle*math.pi)/180)
			#=B14 + 5 * SIN(G14)
			updated_base_y_coord=(coord_Y1) * math.sin((targetAngle*math.pi)/180)
			
			targetDistance=math.sqrt(math.pow(coord_A2-updated_base_x_coord, 2)+math.pow(coord_Y2-updated_base_y_coord, 2))
			#print targetDistance
			
			legServoValues = servoCal2(targetDistance,40) #bodyHeight,footDistance
			#print legServoValues
			singleServoMove(1,legServoValues[0],20,"GREEN")
			singleServoMove(2,legServoValues[1],20,"GREEN")
			CoxaServo=650.46-(targetAngle/.325)
			singleServoMove(0,CoxaServo,20,"RED") #Move Coxa Servo
			#legServoValues = servoCal2(15,15) #bodyHeight,footDistance
			#singleServoMove(1,legServoValues[0],operatingTime/2,"GREEN")
			#singleServoMove(2,legServoValues[1],operatingTime/2,"GREEN")
			#singleServoMove(0,650,operatingTime,"RED") #Move Coxa Servo

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
	testLegSwing()
	#userInputTest()
	#testBodyMove()
	#testLegMove()


# Start of main application
queue = Queue()
start()
