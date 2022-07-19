from is_wire.core import Channel, Subscription, StatusCode,Logger
import json
import sys
from is_msgs.common_pb2 import Position
if __name__ == "__main__":
# -------------------------- Options ------------------------- 
    cont = True
    logger = Logger("Client - CONSUME")
    config_file = '../../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))    
    config["broker_uri"] = "amqp://guest:guest@10.10.2.30:30000"
    channel = Channel(config["broker_uri"])
    # robot_config = config["robot"]
    robot = sys.argv[1]
    topic = sys.argv[2]
    subscription = Subscription(channel)
    subscription.subscribe(topic=f"ROSTranslator.{config['translator_id']}.Consume.{robot}.{topic}")
    logger.info("Consuming...")
    while True:
        msg = channel.consume()
        if msg.status is None:
            position = msg.unpack(Position)
            print(f"x: {position.x} - y: {position.y}")
        else:
            print(msg.status.why)
            # break