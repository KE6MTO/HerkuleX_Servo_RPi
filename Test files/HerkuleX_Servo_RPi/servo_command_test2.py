#import herkulex
import pyherkulex
#from herkulex import servo
#from pyherkulex import servo

#connect to the serial port
connect("/dev/ttyAMA0",115200)

#scan for servos, it returns a tuple with servo id & model number
servos = pyherkulex.scan_servos()

print servos