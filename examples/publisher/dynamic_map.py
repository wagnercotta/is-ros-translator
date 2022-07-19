from is_wire.core import Channel,Message,Subscription
from is_msgs.common_pb2 import Position
from is_msgs.robot_pb2 import BasicMoveTask
import json
import sys
if __name__ == "__main__":
# -------------------------- Options ------------------------- 
    config_file = '../../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))    
    config["broker_uri"] = "amqp://guest:guest@10.10.2.30:30000"
    pub_channel = Channel(config["broker_uri"])
    con_channel = Channel(config["broker_uri"])
    subscription = Subscription(con_channel)
    topic_ros = "dynamic_map"
    robot = sys.argv[1]
    topic = f"ROSTranslator.{config['translator_id']}.Publish.{robot}.{topic_ros}"
    message = Message(reply_to=subscription)
    print(f"Sending message to: {topic}")
    pub_channel.publish(message,topic=topic)
    reply = con_channel.consume()
    print(reply)
