from is_wire.core import Channel,Message, Subscription
from is_msgs.common_pb2 import Position
from is_msgs.robot_pb2 import BasicMoveTask
import sys
import json
if __name__ == "__main__":
# -------------------------- Options ------------------------- 
    config_file = '../../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))    
    config["broker_uri"] = "amqp://guest:guest@192.168.0.61:5672"
    con_channel = Channel(config["broker_uri"])
    pub_channel = Channel(config["broker_uri"])
    robot = sys.argv[1] if len(sys.argv) > 1 else 'Robot_0'
    topic_ros = "move_base_simple_goal"
    topic = f"ROSTranslator.{config['translator_id']}.Publish.{robot}.{topic_ros}"
    subscription = Subscription(con_channel)
    subscription.subscribe(topic=f"ROSTranslator.{config['translator_id']}.Consume.{robot}.odometry_filtered")
    offset = 0.40
    x = 0.0
    y = 0.0
    reachedGoal = False
    while True:
        min_accept_x = x-offset
        max_accept_x = x+offset
        while not reachedGoal:
            msg = con_channel.consume()
            if msg.status is None:
                position = msg.unpack(Position)
                print(f"Position Received. X = {position.x}, Y = {position.y}")
                if position.x > min_accept_x and position.x < max_accept_x:
                    print("Reached Goal")
                    reachedGoal = True
                    if x == 0.0:
                        x = -9.2
                        y = -1.0
                    else:
                        x = 0.0
                        y = 0.0
        print("Creating BMT Message")
        bmt = BasicMoveTask()
        bmt.positions.extend([Position(x=x, y=y, z=0)])
        message = Message()
        message.pack(bmt)
        print(f"Sending {robot} to x: {x},y:{y}")
        pub_channel.publish(message,topic=topic)
        reachedGoal = False
