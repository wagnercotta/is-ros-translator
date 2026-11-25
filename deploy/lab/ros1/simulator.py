#!/usr/bin/env python3
"""Minimal simulated ROS 1 robot for mirroring Create 3 E2E waypoints.

The simulator is a normal ROS 1 /move_base action endpoint. It has no IS-Wire
client or ROS Translator logic. The external E2E client orchestrates both robots.
"""

import math
import threading

import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_msgs.srv import GetMap, GetMapResponse
from visualization_msgs.msg import Marker


class SimulatedRobot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.lock = threading.Lock()
        self.current_goal = None
        self.travelled = []

        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=20)
        self.pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.pub_path = rospy.Publisher('/ros1_sim/path', Path, queue_size=5, latch=True)
        self.pub_goal = rospy.Publisher('/ros1_sim/goal_marker', Marker, queue_size=5, latch=True)
        self.tf = tf2_ros.TransformBroadcaster()
        self.map = self.make_map()
        self.pub_map.publish(self.map)
        self.map_service = rospy.Service('/dynamic_map', GetMap, self.get_map)
        self.server = actionlib.SimpleActionServer(
            '/move_base', MoveBaseAction, execute_cb=self.navigate, auto_start=False
        )
        self.server.start()
        rospy.Timer(rospy.Duration(0.1), self.publish_state)
        rospy.loginfo('ROS1 simulated robot: /move_base, /odom, /map, /ros1_sim/path')

    @staticmethod
    def make_map():
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.10
        grid.info.width = 70
        grid.info.height = 70
        grid.info.origin.position.x = -3.5
        grid.info.origin.position.y = -3.5
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (70 * 70)
        return grid

    def get_map(self, _):
        self.map.header.stamp = rospy.Time.now()
        return GetMapResponse(self.map)

    def publish_state(self, _):
        with self.lock:
            x, y = self.x, self.y
        stamp = rospy.Time.now()

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = stamp
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(odom)

        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.header.stamp = stamp
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)

        with self.lock:
            # Keep enough samples to display the five cycles without unlimited growth.
            if not self.travelled or math.hypot(x - self.travelled[-1][0], y - self.travelled[-1][1]) > 0.01:
                self.travelled.append((x, y))
                self.travelled = self.travelled[-2000:]
            points = list(self.travelled)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for px, py in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_path.publish(path)

    def publish_goal(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'mirrored_create3_goal'
        marker.id = 1
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 0.3
        marker.color.a = 1.0
        self.pub_goal.publish(marker)

    def navigate(self, goal):
        target = goal.target_pose.pose.position
        x_target, y_target = target.x, target.y
        if not (math.isfinite(x_target) and math.isfinite(y_target)):
            self.server.set_aborted(MoveBaseResult(), 'Non-finite waypoint')
            return
        self.publish_goal(x_target, y_target)
        rate = rospy.Rate(10)
        for _ in range(25):
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                return
            with self.lock:
                self.x += (x_target - self.x) * 0.25
                self.y += (y_target - self.y) * 0.25
                current_x, current_y = self.x, self.y
            feedback = MoveBaseFeedback()
            feedback.base_position.header.frame_id = 'map'
            feedback.base_position.header.stamp = rospy.Time.now()
            feedback.base_position.pose.position.x = current_x
            feedback.base_position.pose.position.y = current_y
            feedback.base_position.pose.orientation.w = 1.0
            self.server.publish_feedback(feedback)
            rate.sleep()

        with self.lock:
            self.x, self.y = x_target, y_target
        self.server.set_succeeded(MoveBaseResult())


if __name__ == '__main__':
    rospy.init_node('ros1_simulated_robot')
    SimulatedRobot()
    rospy.spin()
