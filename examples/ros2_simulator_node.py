#!/usr/bin/env python3
"""ROS 2 Jazzy protocol-level robot used by the gateway's Docker E2E lab.

This is a deterministic kinematic demonstration, NOT Nav2/Gazebo or collision
avoidance. It provides a Nav2-compatible action, a demo PoseStamped goal topic,
real ROS 2 odometry, TF, Path, Marker and occupancy-map publications for RViz.
"""
from __future__ import annotations
import json
import math
import threading
import time
import traceback

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from nav_msgs.srv import GetMap
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def yaw_quaternion(yaw: float):
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class DemoRobot(Node):
    SPEED_MPS = 0.7
    POSITION_TOLERANCE = 0.035

    def __init__(self):
        super().__init__('pris_gateway_demo_robot')
        self.lock = threading.RLock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.target = None
        self.target_yaw = 0.0
        self.goal_generation = 0
        self.goal_finished_generation = 0
        self.trail = []
        self.last_tick = time.monotonic()
        self.last_path_publish = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/pris_demo/path', latched)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.marker_pub = self.create_publisher(Marker, '/pris_demo/marker', 10)
        self.state_pub = self.create_publisher(String, '/pris_demo/state', 10)
        self.chatter = self.create_publisher(String, '/chatter', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.goal_sub = self.create_subscription(PoseStamped, '/pris_demo/goal',
                                                 self.on_goal_topic, 10)
        self.server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self.navigate_action,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.map_service = self.create_service(GetMap, '/map_server/map', self.get_map)
        self.map_msg = self.make_map()
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_map)
        self.create_timer(1.0, lambda: self.chatter.publish(String(data='ROS2 lab heartbeat')))
        self.get_logger().info(
            'E2E robot ready: /pris_demo/goal, /pris_demo/path, /pris_demo/marker, '
            '/pris_demo/state, /tf, /odom, /map, /navigate_to_pose, /map_server/map'
        )

    def make_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = 0.5
        msg.info.width = 20
        msg.info.height = 20
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -5.0
        msg.info.origin.orientation.w = 1.0
        # Artificial occupancy grid: border walls and a reference square.
        cells = [0] * 400
        for y in range(20):
            for x in range(20):
                if x in (0, 19) or y in (0, 19):
                    cells[y * 20 + x] = 100
                elif x in (15, 16) and y in (15, 16):
                    cells[y * 20 + x] = 65
        msg.data = cells
        return msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def get_map(self, _request, response):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        response.map = self.map_msg
        return response

    def emit(self, state: str, generation: int):
        msg = String()
        msg.data = json.dumps({'state': state, 'generation': generation,
                               'x': round(self.x, 4), 'y': round(self.y, 4)})
        self.state_pub.publish(msg)

    def set_target(self, x: float, y: float, yaw: float) -> int:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            raise ValueError('Goal coordinates must be finite')
        if not (-4.0 <= x <= 4.0 and -4.0 <= y <= 4.0):
            raise ValueError('Demo map only allows targets within [-4,4] metres')
        with self.lock:
            self.goal_generation += 1
            generation = self.goal_generation
            self.target = (x, y)
            self.target_yaw = yaw
            self.emit('goal_received', generation)
            self.get_logger().info('Demo goal %d: x=%.2f y=%.2f' % (generation, x, y))
            return generation

    def on_goal_topic(self, msg: PoseStamped):
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warn('Rejected demo goal outside map frame')
            return
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            self.set_target(float(p.x), float(p.y), yaw)
        except ValueError as exc:
            self.get_logger().warn('Rejected demo goal: %s' % exc)

    def navigate_action(self, goal_handle):
        try:
            return self._navigate_action(goal_handle)
        except Exception:
            self.get_logger().error('NavigateToPose callback crashed:\n' + traceback.format_exc())
            if goal_handle.is_active:
                goal_handle.abort()
            return NavigateToPose.Result()

    def _navigate_action(self, goal_handle):
        pose = goal_handle.request.pose
        if pose.header.frame_id not in ('', 'map'):
            self.get_logger().error('NavigateToPose aborted: unsupported frame %r' % pose.header.frame_id)
            goal_handle.abort()
            return NavigateToPose.Result()
        p, q = pose.pose.position, pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            generation = self.set_target(p.x, p.y, yaw)
        except ValueError as exc:
            self.get_logger().error('NavigateToPose aborted: invalid goal: %s' % exc)
            goal_handle.abort()
            return NavigateToPose.Result()
        self.get_logger().info('NavigateToPose action started: generation=%d target=(%.2f, %.2f)' %
                               (generation, p.x, p.y))
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('NavigateToPose canceled: generation=%d' % generation)
                with self.lock:
                    if generation == self.goal_generation:
                        self.target = None
                goal_handle.canceled()
                return NavigateToPose.Result()
            with self.lock:
                superseded = self.goal_generation != generation
                distance = math.hypot(p.x - self.x, p.y - self.y)
                finished = self.goal_finished_generation == generation
                x, y, current_yaw = self.x, self.y, self.yaw
            if superseded:
                self.get_logger().warn('NavigateToPose aborted: goal generation %d superseded by %d' %
                                       (generation, self.goal_generation))
                goal_handle.abort()
                return NavigateToPose.Result()
            if finished or distance <= self.POSITION_TOLERANCE:
                self.get_logger().info('NavigateToPose succeeded: generation=%d' % generation)
                goal_handle.succeed()
                return NavigateToPose.Result()
            fb = NavigateToPose.Feedback()
            fb.current_pose.header.frame_id = 'map'
            fb.current_pose.header.stamp = self.get_clock().now().to_msg()
            fb.current_pose.pose.position.x = x
            fb.current_pose.pose.position.y = y
            z, w = yaw_quaternion(current_yaw)
            fb.current_pose.pose.orientation.z = z
            fb.current_pose.pose.orientation.w = w
            fb.distance_remaining = distance
            goal_handle.publish_feedback(fb)
            time.sleep(0.1)
        self.get_logger().error('NavigateToPose aborted: ROS 2 is shutting down')
        goal_handle.abort()
        return NavigateToPose.Result()

    def tick(self):
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.01), 0.3)
        self.last_tick = now
        with self.lock:
            if self.target is not None:
                delta_x = self.target[0] - self.x
                delta_y = self.target[1] - self.y
                distance = math.hypot(delta_x, delta_y)
                movement = min(distance, self.SPEED_MPS * dt)
                if distance > self.POSITION_TOLERANCE:
                    self.yaw = math.atan2(delta_y, delta_x)
                    self.x += delta_x / distance * movement
                    self.y += delta_y / distance * movement
                else:
                    self.x, self.y = self.target
                    self.yaw = self.target_yaw
                    self.target = None
                    self.goal_finished_generation = self.goal_generation
                    self.emit('goal_reached', self.goal_generation)
            x, y, yaw = self.x, self.y, self.yaw
            target = self.target
            stamp = self.get_clock().now().to_msg()
            z, w = yaw_quaternion(yaw)
            position = PoseStamped()
            position.header.frame_id = 'map'
            position.header.stamp = stamp
            position.pose.position.x, position.pose.position.y = x, y
            position.pose.orientation.z, position.pose.orientation.w = z, w
            if not self.trail or math.hypot(x - self.trail[-1].pose.position.x,
                                            y - self.trail[-1].pose.position.y) > 0.015:
                self.trail.append(position)
                self.trail = self.trail[-240:]
            poses = list(self.trail)

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = stamp
        odom.child_frame_id = 'base_link'
        odom.pose.pose = position.pose
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.header.stamp = stamp
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = z
        tf.transform.rotation.w = w
        self.tf_pub.sendTransform(tf)

        if now - self.last_path_publish >= 0.3:
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = stamp
            path.poses = poses
            self.path_pub.publish(path)
            self.last_path_publish = now

        robot = Marker()
        robot.header.frame_id = 'map'
        robot.header.stamp = stamp
        robot.ns = 'pris_demo'
        robot.id = 0
        robot.type = Marker.ARROW
        robot.action = Marker.ADD
        robot.pose = position.pose
        robot.scale.x, robot.scale.y, robot.scale.z = 0.7, 0.25, 0.25
        robot.color.a = 1.0
        robot.color.r, robot.color.g, robot.color.b = 0.12, 0.65, 0.95
        self.marker_pub.publish(robot)

        if target is not None:
            goal = Marker()
            goal.header.frame_id = 'map'
            goal.header.stamp = stamp
            goal.ns = 'pris_demo'
            goal.id = 1
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position.x, goal.pose.position.y = target
            goal.pose.orientation.w = 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.22
            goal.color.a = 1.0
            goal.color.r, goal.color.g, goal.color.b = 0.95, 0.35, 0.12
            self.marker_pub.publish(goal)
        else:
            clear = Marker()
            clear.header.frame_id = 'map'
            clear.ns = 'pris_demo'
            clear.id = 1
            clear.action = Marker.DELETE
            self.marker_pub.publish(clear)


def main():
    rclpy.init()
    node = DemoRobot()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
