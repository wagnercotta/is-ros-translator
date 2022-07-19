from is_msgs.robot_pb2 import BasicMoveTask 
from geometry_msgs.msg import PoseStamped
from rospy_message_converter import message_converter
from math import pi
from .utils import Position_to_Point

def move_base_simple_goal(is_msg):
    basic_move_task = is_msg.unpack(BasicMoveTask)
    step = basic_move_task.positions[0]
    
    mbag_standard = PoseStamped()
    mbag_standard.header.frame_id = "map"
    
    mbag_standard.pose.position = Position_to_Point(step)
    mbag_standard.pose.orientation.x = 0
    mbag_standard.pose.orientation.y = 0
    mbag_standard.pose.orientation.z = 0
    mbag_standard.pose.orientation.w = 1

    mbag_json = message_converter.convert_ros_message_to_dictionary(mbag_standard)
    return mbag_json,"PoseStamped","Topic"