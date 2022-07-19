import os
from pydoc_data.topics import topics
import sys
import json
import roslibpy
import roslibpy.actionlib
import threading
import time
from importlib import import_module
from is_wire.core import Channel,Subscription,Message
from core.utils import get_object_instance

PUBLISHER = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.dirname(PUBLISHER)
sys.path.append(SOURCE)

   
class RobotConnection(object):
    def __init__(self,ros_translator,robot_config,broker_uri) -> None:
        self.ros_translator = ros_translator
        self.logger = ros_translator.logger
        self.robot_channel = Channel(broker_uri)
        self.robot_pub_channel = Channel(broker_uri)
        self.id = robot_config['id']
        self.name = f"Robot_{self.id}"
        self.subscription = Subscription(self.robot_channel,name=f"{self.name}")
        self.subscription.subscribe(topic=f"{ros_translator.service_name}.{self.name}.*")
        self.ip = robot_config['ip']
        self.port = robot_config['port']
        try:
            self.roslib = roslibpy.Ros(host=str(self.ip), port=int(self.port))
            self.roslib.run(timeout=10)
            self.logger.info(f"{self.name} connected.")
        except:
            self.logger.info(f"Could not connect to {self.name}.")
        self.status = self.roslib.is_connected
        self.worker = self.create_thread().start()
    
    def create_thread(self):
        return threading.Thread(target=self.get_messages)

    def get_messages(self):
        msg = None
        while True:
            self.status = self.roslib.is_connected
            if self.status:
                self.logger.info(f"{self.name} ready.")
                try:
                    msg = self.robot_channel.consume(timeout=0.1)
                    self.logger.info(f"Request received to {self.name}. ")
                except:
                    self.logger.warn("Did not consume message.")
                if msg:
                    self.process_message(msg)
                    msg = None
            else:
                self.logger.error(f"{self.name} not connected. Retrying...")
                time.sleep(5)
    
    def process_message(self,msg):
        translate_function_name = msg.topic.split('.')[-1]
        if not os.path.exists(f"{SOURCE}/translate_functions/{translate_function_name}.py"):
            self.logger.error(f"Could not find file {translate_function_name}.py in translate function folder.")
        else:
            mod = import_module(f"translate_functions.{translate_function_name}")
            translate_function = getattr(mod, translate_function_name)
            self.translate_and_publish(translate_function,msg)

    def get_topics(self):
        if self.status:
            return self.roslib.get_topics()

    def get_services(self):
        if self.status:
            return self.roslib.get_services()

    def translate_and_publish(self, translate_function, message):
        translated_message,message_type,publish_type = translate_function(message)
        dict_obj = get_object_instance('topic_is',message.topic.split('.')[-1],self.ros_translator.correspondence_map,dict)
        if publish_type == "Topic":
            try:
                publisher = roslibpy.Topic(self.roslib, dict_obj['topic_ros'], message_type,latch=True)
                publisher.publish(roslibpy.Message(translated_message))
                publisher.unadvertise()
            except:
                self.logger.error(f"Something went wrong when publishing to {dict_obj['topic']}")
        elif publish_type == "SimpleAction":
            self.logger.info(f"Sending SimpleAction to {dict_obj['topic_ros']}")
            try:
                action_client = roslibpy.actionlib.ActionClient(self.roslib,dict_obj['topic_ros'],message_type)
            except:
                self.logger.warn(f"Could not connect to {self.name} Action Client.")
                return
            goal = roslibpy.actionlib.Goal(action_client,roslibpy.Message(translated_message))
            goal.send()
        elif publish_type == "Service":
            service_client = roslibpy.Service(self.roslib, dict_obj['topic_ros'],message_type)
            request = roslibpy.ServiceRequest(translated_message) if translated_message is not None else roslibpy.ServiceRequest()
            result = service_client.call(request)
            reply_msg = Message()
            if result is not None:
                reply_msg.body = json.dumps(dict(result)).encode()
            print(reply_msg)
            self.robot_pub_channel.publish(message = reply_msg, topic=message.reply_to)
        return