from core.robotconnection import RobotConnection
from core.utils import get_object_instance
import re
import os
import sys
import json
import socket
import requests
from is_wire.core import Channel, Logger,Subscription,Status,StatusCode, Message

CONSUMER = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.dirname(CONSUMER)
sys.path.append(SOURCE)

class ROSTranslator_Consumer(object):
    def __init__(self,translator_id, broker_uri,management_url):
        self.management_url = management_url + "/api/bindings"
        self.broker_uri=broker_uri
        self.translator_id = translator_id
        self.logger = Logger("ROSTranslator_Consumer")
        self.correspondence_map = json.load(open(f"{SOURCE}/translate_functions/correspondence_map.json", 'r'))
        self.robot_connections = []
    
    def query_consumers(self,topic):
        reply = requests.get(self.management_url)
        if reply.status_code != 200:
            why = "Failed to query management API, code={}".format(
                reply.status_code)
            raise RuntimeError(why)
        consumers_count = 0
        bindings = reply.json()
        for bind in bindings:
            if bool(re.search(topic,bind['routing_key'])):
                consumers_count += 1
        return consumers_count
        
    def run(self):
        self.service_name = f"ROSTranslator.{self.translator_id}.Consume"
        self.channel = Channel(self.broker_uri)
        subscription = Subscription(self.channel,name=self.service_name)
        subscription.subscribe(topic="binding.*")
        self.logger.info(f"{self.service_name} listening for consume events...")
        while True:
            try:
                msg = self.channel.consume()
            except socket.io:
                self.logger.warn("Did not consume message.")
            full_topic = msg.metadata["routing_key"]
            if bool(re.search(self.service_name, full_topic)):
                event = msg.topic.split('.')[-1]
                topic_is = re.split('\.',full_topic)[-1]
                requested_robot = re.split('\.',full_topic)[-2].lower()
                
                dict_item = get_object_instance('topic_is',topic_is,self.correspondence_map,dict)
                if dict_item is None:
                    error_message = Message()
                    msg = f"[{self.service_name}] Could not find {topic_is} in correspondence map."
                    error_message.body = msg.encode()
                    error_message.status = Status(StatusCode.NOT_FOUND,why=msg)
                    self.channel.publish(error_message,topic=full_topic)
                    self.logger.error(f"Could not find {topic_is} in correspondence map.")

                else:
                    with open('etc/conf/robots.json') as robots_cfg:
                        robots_config = json.load(robots_cfg)
                    
                    robot_connection = get_object_instance('ip',robots_config[requested_robot]['ip'],self.robot_connections,RobotConnection)
                    if event == "created":
                        if robot_connection is None:
                            robot_connection = RobotConnection(self,robots_config[requested_robot],self.broker_uri)
                            self.robot_connections.append(robot_connection)            
                            robot_connection.worker.start()
                        robot_connection.add_work_done = False
                        robot_connection.add_listener_list.append([dict_item['topic_ros'],dict_item['translate_function'],dict_item['topic_is']])
                        if (robot_connection.status) and (not robot_connection.worker.is_alive()):
                            robot_connection.break_thread = False
                            robot_connection.worker = robot_connection.create_thread()
                            robot_connection.worker.start()
                            while (robot_connection.status) and (not robot_connection.add_work_done):
                                pass
                    elif event == "deleted":
                        if robot_connection is not None:
                            robot_connection.del_work_done = False
                            robot_connection.del_listener_list.append([dict_item['topic_ros'],dict_item['topic_is']])
                            while (robot_connection.status) and (not robot_connection.del_work_done):
                                pass
                            if self.query_consumers(f"{requested_robot.capitalize()}.{topic_is}") == 0:
                                robot_connection.break_thread = True
                                robot_connection.worker.join()
                    robots_cfg.close()


if __name__ == "__main__":
    config_file = '../../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))

    broker_uri = config['broker_uri']
    robots_config_file = config['robots_config_file']
    translator_id = config['translator_id']
    correspondence_map = json.load(open(config['correspondence_map_file'], 'r'))

    service = ROSTranslator_Consumer(translator_id,robots_config_file,correspondence_map,broker_uri=broker_uri)
    service.run()