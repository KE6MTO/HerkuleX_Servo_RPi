#!/usr/bin/env python
import pika
import random
import time

##connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
##channel = connection.channel()
##channel.queue_declare(queue='TLM')


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
