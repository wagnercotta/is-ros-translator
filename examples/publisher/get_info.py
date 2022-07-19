from is_wire.core import Channel,Message,Subscription
import json
if __name__ == "__main__":
# -------------------------- Options ------------------------- 
    config_file = '../../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))    
    config["broker_uri"] = "amqp://guest:guest@10.10.2.30:30000"
    pub_channel = Channel(config["broker_uri"])
    con_channel = Channel(config["broker_uri"])
    subscription = Subscription(con_channel)
    robot = "Robot_0"
    # topic = f"ROSTranslator.{config['translator_id']}.Info.{robot}.Topics"
    topic = f"ROSTranslator.{config['translator_id']}.Info.{robot}.Services"
    message = Message(reply_to=subscription)
    print(f"Sending message to: {topic}")
    pub_channel.publish(message,topic=topic)
    msg = con_channel.consume()
    print(msg)
