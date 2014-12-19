

ser = serial.Serial('/dev/ttyAMA0', 9600)

def logToSQL(sqldata):
	try:
		currentTime = strftime("%Y-%m-%d %H:%M:%S")
		con = _mysql.connect('localhost','root','squirrel2010'.'nodedata')
		con.query("""INSERT INTO data(node_id, data1,data2,data3,data4,data5,data6,timestamp) VALUES (%s,'%s','%s','%s','%s','%s','%s','%s')""" %(data['Node'])
		print("--> DB Insert Good")
		
	except:
		print("----> Failed to write DB")
		pass
		
	finally:
		if con:
			con.close()

def logToDisk(data):
	try:
		f = open('datalog.txt','a')
		f.write(data)
		print("--> Log to file good")
		f.close()
		
	except:
		print("----> Failed to write log")
		pass