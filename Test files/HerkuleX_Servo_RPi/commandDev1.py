#!/usr/bin/env python
import pika
import time

rabbitMQHost = 'localhost'

connection = pika.BlockingConnection(pika.ConnectionParameters(host=rabbitMQHost))
channel = connection.channel()

channel.queue_declare(queue='commandData')
channel.queue_declare(queue='telemetryData')

print ' [*] Waiting for messages. To exit press CTRL+C'

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
		
		# print " [x] Received %r" % (body,)




channel.basic_consume(callback,queue='commandData',no_ack=True)
channel.start_consuming()
