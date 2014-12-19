import os
import glob
import time
import requests
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '10*')[0]
device_file = device_folder + '/w1_slave'
def read_temp_raw():
	f = open(device_file, 'r')
	lines = f.readlines()
	f.close()
	return lines
def read_temp():
	# Open the file that we viewed earlier so that python can see what is in it. Replace the serial number as before. 
	tfile = open("/sys/bus/w1/devices/10-000801f9e4f9/w1_slave") 
	# Read all of the text in the file. 
	text = tfile.read() 
	# Close the file now that the text has been read. 
	tfile.close() 
	# Split the text with new lines (\n) and select the second line. 
	secondline = text.split("\n")[1] 
	# Split the line into words, referring to the spaces, and select the 10th word (counting from 0). 
	temperaturedata = secondline.split(" ")[9] 
	# The first two characters are "t=", so get rid of those and convert the temperature from a string to a number. 
	temperature = float(temperaturedata[2:]) 
	# Put the decimal point in the right place and display it. 
	temperature = temperature / 1000 
	temp_f = temperature * 9.0 / 5.0 + 32.0
	#return temperature, temp_f
	return temp_f
 
while True:
	current_temp=read_temp()
	#print(current_temp) 
	#temp_f = read_temp(1)
	#print(temp_f)
	current_temp_string = str(current_temp)
	print(current_temp_string)
	r = requests.put("http://10.90.30.75:8081/rest/items/RaspberryPI_1_temp1/state", current_temp_string)
	print(r.content)
	time.sleep(60)

# Open the file that we viewed earlier so that python can see what is in it. Replace the serial number as before. 
# tfile = open("/sys/bus/w1/devices/10-000802824e58/w1_slave") 
# Read all of the text in the file. 
# text = tfile.read() 
# Close the file now that the text has been read. 
# tfile.close() 
# Split the text with new lines (\n) and select the second line. 
# secondline = text.split("\n")[1] 
# Split the line into words, referring to the spaces, and select the 10th word (counting from 0). 
# temperaturedata = secondline.split(" ")[9] 
# The first two characters are "t=", so get rid of those and convert the temperature from a string to a number. 
# temperature = float(temperaturedata[2:]) 
# Put the decimal point in the right place and display it. 
# temperature = temperature / 1000 
# print temperature