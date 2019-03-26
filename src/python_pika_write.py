import pika
import os
import fileinput

rabbit_host = os.getenv('rabbit_host', "localhost")
rabbit_port = os.getenv('rabbit_port', 5672)
rabbit_user = os.getenv('rabbit_user', "guest")
rabbit_password = os.getenv('rabbit_password', "guest")
rabbit_queue = os.getenv('rabbit_queue', "updates")

credentials = pika.PlainCredentials(rabbit_user, rabbit_password)
parameters = pika.ConnectionParameters(rabbit_host,
                                   rabbit_port,
                                   '/',
                                   credentials)
connection = pika.BlockingConnection(parameters)
channel = connection.channel()

channel.exchange_declare(
    exchange=rabbit_queue,
    exchange_type="fanout"
    )

for line in fileinput.input():
  # print("line: " + line)
  channel.basic_publish(exchange=rabbit_queue,
                    routing_key='',
                    body=line)

channel.start_consuming()
connection.close()

