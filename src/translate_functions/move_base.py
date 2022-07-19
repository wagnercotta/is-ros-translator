from move_base_msgs.msg import MoveBaseGoal
from rospy_message_converter import message_converter
from is_msgs.robot_pb2 import BasicMoveTask
from math import pi
from .utils import Yaw_to_Quaternion



def move_base(is_msg):
    bmt = is_msg.unpack(BasicMoveTask)
    step = bmt.positions[0]

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.pose.position.x = step.x
    goal.target_pose.pose.position.y = step.y
    quart = Yaw_to_Quaternion(8*pi/180)
    goal.target_pose.pose.orientation.x = quart[0]
    goal.target_pose.pose.orientation.y = quart[1]
    goal.target_pose.pose.orientation.z = quart[2]
    goal.target_pose.pose.orientation.w = quart[3]
    converted_message = message_converter.convert_ros_message_to_dictionary(goal)
    return converted_message,"MoveBase","SimpleAction"