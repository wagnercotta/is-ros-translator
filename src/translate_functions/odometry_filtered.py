from is_wire.core import Message
from is_msgs.common_pb2 import Position

def odometry_filtered(message):
    ros_position = message['pose']['pose']['position']
    position = Position()
    position.x, position.y,position.z = ros_position['x'],ros_position['y'],ros_position['z']
    msg = Message(content = position)
    return msg