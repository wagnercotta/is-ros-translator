from is_wire.core import Channel,Message
from is_msgs.common_pb2 import Position
from is_msgs.robot_pb2 import BasicMoveTask
import json
import sys
if __name__ == "__main__":
# -------------------------- Options ------------------------- 
    config_file = '../../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))    
    config["broker_uri"] = "amqp://guest:guest@10.10.2.30:30000"
    channel = Channel(config["broker_uri"])
    topic_ros = "move_base"
    robot = sys.argv[1]
    point_x = float(sys.argv[2])
    point_y = float(sys.argv[3])
    topic = f"ROSTranslator.{config['translator_id']}.Publish.{robot}.{topic_ros}"
    bmt = BasicMoveTask()
    bmt.positions.extend([Position(x=point_x, y=point_y, z=0)])
    message = Message()
    message.pack(bmt)
    print(f"Sending message to: {topic}")
    channel.publish(message,topic=topic)
