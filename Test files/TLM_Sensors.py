#!/usr/bin/env python
import pika
import random
import time
import smbus


##connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
##channel = connection.channel()
##channel.queue_declare(queue='TLM')

i2c_bus = smbus.SMBus(1)
# I2C address for LSM303, MAG & ACC                                                     
mag_addr = 0x1E
acc_addr = 0x18
  # LSM303DLHC Registers								Default		Type
LSM303DLHC_REGISTER_ACCEL_CTRL_REG1_A		= 0x20	# 00000111   rw
LSM303DLHC_REGISTER_ACCEL_CTRL_REG2_A		= 0x21	# 00000000   rw
LSM303DLHC_REGISTER_ACCEL_CTRL_REG3_A		= 0x22	# 00000000   rw
LSM303DLHC_REGISTER_ACCEL_CTRL_REG4_A		= 0x23	# 00000000   rw
LSM303DLHC_REGISTER_ACCEL_CTRL_REG5_A		= 0x24	# 00000000   rw
LSM303DLHC_REGISTER_ACCEL_CTRL_REG6_A		= 0x25	# 00000000   rw
LSM303DLHC_REGISTER_ACCEL_REFERENCE_A		= 0x26	# 00000000   r
LSM303DLHC_REGISTER_ACCEL_STATUS_REG_A		= 0x27	# 00000000   r
LSM303DLHC_REGISTER_ACCEL_OUT_X_L_A			= 0x28
LSM303DLHC_REGISTER_ACCEL_OUT_X_H_A			= 0x29
LSM303DLHC_REGISTER_ACCEL_OUT_Y_L_A			= 0x2A
LSM303DLHC_REGISTER_ACCEL_OUT_Y_H_A			= 0x2B
LSM303DLHC_REGISTER_ACCEL_OUT_Z_L_A			= 0x2C
LSM303DLHC_REGISTER_ACCEL_OUT_Z_H_A			= 0x2D
LSM303DLHC_REGISTER_ACCEL_FIFO_CTRL_REG_A	= 0x2E
LSM303DLHC_REGISTER_ACCEL_FIFO_SRC_REG_A	= 0x2F
LSM303DLHC_REGISTER_ACCEL_INT1_CFG_A		= 0x30
LSM303DLHC_REGISTER_ACCEL_INT1_SOURCE_A		= 0x31
LSM303DLHC_REGISTER_ACCEL_INT1_THS_A		= 0x32
LSM303DLHC_REGISTER_ACCEL_INT1_DURATION_A	= 0x33
LSM303DLHC_REGISTER_ACCEL_INT2_CFG_A		= 0x34
LSM303DLHC_REGISTER_ACCEL_INT2_SOURCE_A		= 0x35
LSM303DLHC_REGISTER_ACCEL_INT2_THS_A		= 0x36
LSM303DLHC_REGISTER_ACCEL_INT2_DURATION_A	= 0x37
LSM303DLHC_REGISTER_ACCEL_CLICK_CFG_A		= 0x38
LSM303DLHC_REGISTER_ACCEL_CLICK_SRC_A		= 0x39
LSM303DLHC_REGISTER_ACCEL_CLICK_THS_A		= 0x3A
LSM303DLHC_REGISTER_ACCEL_TIME_LIMIT_A		= 0x3B
LSM303DLHC_REGISTER_ACCEL_TIME_LATENCY_A	= 0x3C
LSM303DLHC_REGISTER_ACCEL_TIME_WINDOW_A		= 0x3D

LSM303DLHC_REGISTER_MAG_CRA_REG_M			= 0x00
LSM303DLHC_REGISTER_MAG_CRB_REG_M			= 0x01
LSM303DLHC_REGISTER_MAG_MR_REG_M			= 0x02
LSM303DLHC_REGISTER_MAG_OUT_X_H_M			= 0x03
LSM303DLHC_REGISTER_MAG_OUT_X_L_M			= 0x04
LSM303DLHC_REGISTER_MAG_OUT_Z_H_M			= 0x05
LSM303DLHC_REGISTER_MAG_OUT_Z_L_M			= 0x06
LSM303DLHC_REGISTER_MAG_OUT_Y_H_M			= 0x07
LSM303DLHC_REGISTER_MAG_OUT_Y_L_M			= 0x08
LSM303DLHC_REGISTER_MAG_SR_REG_Mg			= 0x09
LSM303DLHC_REGISTER_MAG_IRA_REG_M			= 0x0A
LSM303DLHC_REGISTER_MAG_IRB_REG_M			= 0x0B
LSM303DLHC_REGISTER_MAG_IRC_REG_M			= 0x0C
LSM303DLHC_REGISTER_MAG_TEMP_OUT_H_M		= 0x31
LSM303DLHC_REGISTER_MAG_TEMP_OUT_L_M		= 0x32

def twos_comp(val, bits):
	#"compute the 2's compliment of int value val"
	if( (val&(1<<(bits-1))) != 0 ):
		val = val - (1<<bits)
	return val
	return 0

try :
   print 'Attempting to connect to the LSM303'
   i2c_bus.write_byte_data( mag_addr,  0xff, ( 0xff ) >> 8 )
   
   # Enable the accelerometer
   # 100Hz measurement, normal mode, XYZ enabled
   i2c_bus.write_byte_data( acc_addr, LSM303DLHC_REGISTER_ACCEL_CTRL_REG1_A, 0x57)
   accelDataRate = 0b0101
   accelLowPower = 0b0
   accelAxes = 0b111
   
   # continuous update, little endian, +-2G, high resolution disable, 4 wire serial
   i2c_bus.write_byte_data( acc_addr, LSM303DLHC_REGISTER_ACCEL_CTRL_REG4_A, 0x00)
   accelScale = 0b00
   accelHiRez = 0b0
   accelFactor = 0.001
	
   # Enable the magnetometer
   # continuous conversion mode
   i2c_bus.write_byte_data( mag_addr, LSM303DLHC_REGISTER_MAG_MR_REG_M, 0x00)
   # temperature unabled, 15Hz minimum output rate
   i2c_bus.write_byte_data( mag_addr, LSM303DLHC_REGISTER_MAG_CRA_REG_M, 0x10)
   tempEnabled = 0b0
   magDataRate = 0b100
   # sensor input field range +-1.9 gauss
   i2c_bus.write_byte_data( mag_addr, LSM303DLHC_REGISTER_MAG_CRB_REG_M, 0x40)
   magFactor = 1 / 855.0  

   #"Reads the temperature data from the sensor"
   tlo = i2c_bus.read_word_data( mag_addr,LSM303DLHC_REGISTER_MAG_TEMP_OUT_L_M)
   print "DBG: Temp lo: 0x%04X (%d)" % (tlo & 0xFFFF, tlo)
   print tlo
   thi = i2c_bus.read_word_data( mag_addr,LSM303DLHC_REGISTER_MAG_TEMP_OUT_H_M)
   print "DBG: Temp hi: 0x%04X (%d)" % (thi & 0xFFFF, thi)
   print thi
   temp = ((thi << 8) + tlo) >> 4  
   print temp 
   print "Deg C: " + str((twos_comp(temp, 12) / 8.0) + 18)
   print "Deg F: " + str((((twos_comp(temp, 12) / 8.0) + 18) * 1.8) + 32)
   #print (thi / 16) + (tlo / 256)

   
except IOError :
   print 'Error connecting to the LSM303'
   error = 1
else :
   # Now read from LSM303
   print 'Connected to the LSM303'

credentials = pika.PlainCredentials('pi', 'raspberry')
connection = pika.BlockingConnection(pika.ConnectionParameters('10.90.30.129', 5672, "/",  credentials))
channel = connection.channel()

channel.queue_declare(queue='TLM', arguments={
  'x-message-ttl' : 60000})


while True:
	ticks = time.time()
	packet = '$telm,' + str(random.uniform(6, 8)) + ',' + str(random.uniform(1, 4)) + ',' + str(random.uniform(4, 6)) + ',' + str(random.uniform(2, 4)) + ',' + str(ticks) + ',#'
	channel.basic_publish(exchange='', routing_key='TLM', body=packet)
	print packet
	#time.sleep(random.uniform(1, 3))
	time.sleep(1)

connection.close()