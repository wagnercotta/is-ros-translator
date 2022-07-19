import os
import sys
import roslibpy
import threading
from importlib import import_module
from core.utils import get_object_instance
from core.rostopiclistener import ROSTopicListener
from is_wire.core import Logger,Channel,Status,StatusCode, Message

CONSUMER = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.dirname(CONSUMER)
sys.path.append(SOURCE)

class RobotConnection(object):
    def __init__(self,ros_translator,robot_config,broker_uri) -> None:
        self.logger = ros_translator.logger
        self.ros_translator = ros_translator
        self.pub_channel = Channel(broker_uri)
        self.id = robot_config['id']
        self.name = f"robot_{self.id}"
        self.ip = robot_config['ip']
        self.ros_listeners = []
        self.add_listener_list = []
        self.add_work_done = False
        self.del_listener_list = []
        self.del_work_done = False
        self.worker = self.create_thread()
        self.port = robot_config['port']
        self.break_thread = False
        try:
            print(self.ip)
            self.roslib = roslibpy.Ros(host=str(self.ip), port=int(self.port))
            self.roslib.run(timeout=10)
            self.logger.info(f"{self.name} connected.")
        except:
            # message = f"[{self.ros_translator.service_name}] Could not connect to {self.name}."
            # self.return_error_message(message,StatusCode.NOT_FOUND,f"{self.ros_translator.service_name}.{self.name.capitalize()}.*")
            self.logger.info(f"Could not connect to {self.name}.")
        self.status = self.roslib.is_connected
    
    def create_thread(self):
        return threading.Thread(target=self.process_work)

    def process_work(self):
        while not self.break_thread:
            self.status = self.roslib.is_connected
            if self.status:
                for item in self.add_listener_list[:]:
                    if self.add_listener(item[0],item[1],item[2]) != False:
                        self.add_listener_list.remove(item)
                self.add_work_done = True
                for item in self.del_listener_list[:]:
                    if self.del_listener(item[0],item[1]) != False:
                        self.del_listener_list.remove(item)
                self.del_work_done = True

       
    def add_listener(self,topic_ros,translate_function,topic_is):
        ros_listener = get_object_instance('topic_ros',topic_ros,self.ros_listeners,ROSTopicListener)
        full_topic = ".".join([self.ros_translator.service_name,self.name.capitalize(),topic_is])
        if ros_listener is None:
            try:
                message_type = self.roslib.get_topic_type(topic_ros)
            except Exception:
                self.logger.error(f"Could not find topic {topic_ros} in {self.name}")
                message = f"[{self.ros_translator.service_name}] Could not find topic {topic_ros} in {self.name}."
                self.return_error_message(message,StatusCode.NOT_FOUND,full_topic)
                return False

            if not os.path.exists(f"{SOURCE}/translate_functions/{translate_function}.py"):
                self.ros_translator.logger.error(f"Could not find file {translate_function}.py in translate function folder.")
                message = f"[{self.ros_translator.service_name}] Could not find file {translate_function}.py in translate function folder."
                self.return_error_message(message,StatusCode.NOT_FOUND,full_topic)
                return False
            else:
                mod = import_module(f"translate_functions.{translate_function}")
                translate_function = getattr(mod, translate_function)

            ros_listener = ROSTopicListener(self,topic_ros,message_type,translate_function,full_topic)
            ros_listener.subscribe()
            self.ros_listeners.append(ros_listener)
            return True
        else:
            return
    
    def del_listener(self,topic_ros,topic_is):
        ros_listener = get_object_instance('topic_ros',topic_ros,self.ros_listeners,ROSTopicListener)
        if ros_listener is not None:
            if not self.ros_translator.query_consumers(f"{self.name.capitalize()}.{topic_is}") > 0:
                ros_listener.unsubscribe()
                self.ros_listeners.remove(ros_listener)
                del ros_listener
                return True
        else:
            return
 
    def return_error_message(self,message,statuscode,topic_is):
        error_message = Message()
        error_message.body = message.encode()
        error_message.status = Status(statuscode,why=message)
        self.pub_channel.publish(error_message,topic=topic_is)

    def process_message(self,message,translate_function,topic_is):
        msg = translate_function(message)
        self.pub_channel.publish(msg,topic=topic_is)

    def disconnect(self):
        try:
            self.roslib.close()
            while self.roslib.is_connected:
                pass
            self.logger.info(f"Connection of {self.name} shutdown.")
            self.status = self.roslib.is_connected
        except:
            pass
    
    def reconnect(self):
        try:
            self.roslib.connect()
            while not self.roslib.is_connected:
                pass
        except:
            self.logger.error(f"{self.name} could not connect.")
        self.status = self.roslib.is_connected
    
    def __del__(self):
        self.roslib.close()
        self.logger.info(f"{self.name} connection destroyed.")