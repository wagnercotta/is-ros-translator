from core.robotconnection import RobotConnection
import json
import socket
import os
import sys
from core.utils import get_object_instance
from is_wire.core import Channel, Logger,Subscription,Message,Status,StatusCode

CONSUMER = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.dirname(CONSUMER)
sys.path.append(SOURCE)

class ROSTranslator_Publisher(object):
    def __init__(self,translator_id, broker_uri):
        self.broker_uri=broker_uri
        self.translator_id = translator_id
        self.logger = Logger("ROSTranslator_Publisher")
        self.correspondence_map = json.load(open(f"{SOURCE}/translate_functions/correspondence_map.json", 'r'))
        self.robot_connections = []

    def get_topics(self,robot):
        reply_dict = {}
        status = StatusCode.NOT_FOUND
        why = ""
        for connection in self.robot_connections:
            if (robot == '*') or (connection.name == robot.lower()):
                robot_obj = get_object_instance('name',connection.name,self.robot_connections,RobotConnection)
                if robot_obj is not None:
                    if robot_obj.status:
                        try:
                            reply_dict[robot_obj.name] = robot_obj.get_topics()
                            status = StatusCode.OK
                        except:
                            status = StatusCode.DEADLINE_EXCEEDED
                            why = f"No service response received."
                    else:
                        why = f"{robot_obj.name} is offline."
        status = Status(status,why=why)
        return reply_dict,status

    def get_services(self,robot):
        reply_dict = {}
        status = StatusCode.NOT_FOUND
        why = ""
        for connection in self.robot_connections:
            if (robot == '*') or (connection.name == robot.lower()):
                robot_obj = get_object_instance('name',connection.name,self.robot_connections,RobotConnection)
                if robot_obj is not None:
                    if robot_obj.status:
                        try:
                            reply_dict[robot_obj.name] = robot_obj.get_services()
                            status = StatusCode.OK
                        except:
                            status = StatusCode.DEADLINE_EXCEEDED
                            why = f"No service response received."
                    else:
                        why = f"{robot_obj.name()} is offline."
        status = Status(status,why=why)
        return reply_dict,status
    
    def process_message(self,function,robot):
        msg = Message()
        if function == 'Topics':
            info,status = self.get_topics(robot)
        elif function == 'Services':
            info,status = self.get_services(robot)
        msg.status = status
        msg.body = json.dumps(info).encode()
            
        return msg

    def run(self):
        self.service_name = f"ROSTranslator.{self.translator_id}.Publish"
        translator_channel = Channel(self.broker_uri)
        subscription = Subscription(translator_channel)
        subscription.subscribe(topic=f"{self.service_name.replace('.Publish','')}.Info.*.*")
        with open('etc/conf/robots.json') as robots_cfg:
            robots_config = json.load(robots_cfg)
            for robot in robots_config:
                robot_connection = RobotConnection(self,robots_config[robot],self.broker_uri)
                self.robot_connections.append(robot_connection)
        self.logger.info(f"{self.service_name} listening for requests")
        while True:
            try:
                msg = translator_channel.consume()
            except socket.io:
                self.logger.warn("Did not consume message.")
            robot = msg.topic.split('.')[-2]
            function = msg.topic.split('.')[-1]
            content = self.process_message(function,robot)
            translator_channel.publish(content, topic=msg.reply_to)



if __name__ == "__main__":
    config_file = '../../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))

    broker_uri = config['broker_uri']
    robots_config_file = config['robots_config_file']
    translator_id = config['translator_id']
    correspondence_map = json.load(open(config['correspondence_map_file'], 'r'))

    service = ROSTranslator_Publisher(translator_id,robots_config_file,correspondence_map,broker_uri=broker_uri)
    service.run()