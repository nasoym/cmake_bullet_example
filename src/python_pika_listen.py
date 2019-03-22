import pika
import os

rabbit_host = os.getenv('rabbit_host', "localhost")
rabbit_port = os.getenv('rabbit_port', 5672)
rabbit_user = os.getenv('rabbit_user', "guest")
rabbit_password = os.getenv('rabbit_password', "guest")
rabbit_queue = os.getenv('rabbit_queue', "commands")

credentials = pika.PlainCredentials(rabbit_user, rabbit_password)
parameters = pika.ConnectionParameters(rabbit_host,
                                   rabbit_port,
                                   '/',
                                   credentials)
connection = pika.BlockingConnection(parameters)
channel = connection.channel()

def callback(ch, method, properties, body):
  print(body.decode('UTF-8'))

channel.queue_declare(queue=rabbit_queue)
# result = channel.queue_declare(exclusive=True)
# queue_name = result.method.queue
# channel.queue_bind(exchange='infos',queue=queue_name)
channel.basic_consume(callback,
											queue=rabbit_queue,
											no_ack=True)

channel.start_consuming()
connection.close()

