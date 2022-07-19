from is_wire.core import Channel, Logger, Status, StatusCode,Subscription,Message
import roslibpy

class ROSTopicListener(object):
    def __init__(self,robot_connection,topic_ros,message_type,translate_function,full_topic):
        self.logger = robot_connection.ros_translator.logger
        self.topic_ros = topic_ros
        self.robot_connection = robot_connection
        self.translate_function = translate_function
        self.full_topic = full_topic
        self.listener = roslibpy.Topic(self.robot_connection.roslib, self.topic_ros, message_type,reconnect_on_close=True)
        self.consumers_count = 0
        self.status = ""
        self.logger.info(f"Listener for topic {self.topic_ros} created.")
    
    def subscribe(self):
        try:
            self.listener.subscribe(lambda message: self.robot_connection.process_message(message,self.translate_function,self.full_topic))
            self.status = "subscribed"
            self.logger.info(f"Subscribed to {self.topic_ros}.")
        except:
            self.logger.info(f"Something happened.")

    def unsubscribe(self):
        self.status = "unsubscribed"
        self.listener.unsubscribe()
        self.logger.info(f"Unsubscribed to {self.topic_ros}.")

    def __str__(self) -> str:
        return f"Topic: {self.topic_ros} - Status: {self.status} - Consumers: {self.consumers_count}"
    
    def __del__(self):
        self.logger.info(f"Listener for topic {self.topic_ros} destroyed.")
