#!/usr/bin/env python3
"""Minimal ROS 1 Noetic demonstration node (requires installed ROS 1).

Implements a simulated action server /move_base, /odom and /dynamic_map.
No Gazebo, dynamics or planning; smoke test only.
"""
import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult, MoveBaseFeedback
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.srv import GetMap, GetMapResponse
from std_msgs.msg import String


class DemoRobot:
    def __init__(self):
        self.x, self.y = 0.0, 0.0
        self.pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.chatter = rospy.Publisher("/chatter", String, queue_size=10)
        rospy.Timer(rospy.Duration(1), lambda _: self.chatter.publish(String(data="ROS1 lab heartbeat")))
        self.map = rospy.Service("/dynamic_map", GetMap, self.get_map)
        self.server = actionlib.SimpleActionServer("/move_base", MoveBaseAction,
                                                    execute_cb=self.navigate, auto_start=False)
        self.server.start()
        rospy.Timer(rospy.Duration(0.2), self.publish_odom)
        rospy.loginfo("Fake ROS1 robot ready: /move_base /odom /dynamic_map")

    def publish_odom(self, _):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.w = 1.0
        self.pub.publish(msg)

    def get_map(self, _):
        m = OccupancyGrid()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.info.resolution = 0.2
        m.info.width = 20
        m.info.height = 20
        m.info.origin.orientation.w = 1.0
        m.data = [0] * 400
        return GetMapResponse(m)

    def navigate(self, goal):
        target = goal.target_pose.pose.position
        rate = rospy.Rate(10)
        for _ in range(20):
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                return
            self.x += (target.x - self.x) * 0.2
            self.y += (target.y - self.y) * 0.2
            feedback = MoveBaseFeedback()
            feedback.base_position.header.frame_id = "map"
            feedback.base_position.pose.position.x = self.x
            feedback.base_position.pose.position.y = self.y
            feedback.base_position.pose.orientation.w = 1
            self.server.publish_feedback(feedback)
            rate.sleep()
        self.x, self.y = target.x, target.y
        self.server.set_succeeded(MoveBaseResult())


if __name__ == "__main__":
    rospy.init_node("pris_gateway_demo_robot")
    DemoRobot()
    rospy.spin()
