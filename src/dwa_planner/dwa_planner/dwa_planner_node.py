#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import numpy as np

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # Declare parameters
        self.declare_parameter('max_speed', 0.22)
        self.declare_parameter('min_speed', -0.22)
        self.declare_parameter('max_yaw_rate', 2.0)
        self.declare_parameter('max_accel', 0.5)
        self.declare_parameter('max_dyaw_rate', 3.0)
        self.declare_parameter('v_reso', 0.05)
        self.declare_parameter('yaw_rate_reso', 0.1)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('predict_time', 3.0)
        self.declare_parameter('to_goal_cost_gain', 1.0)
        self.declare_parameter('obstacle_cost_gain', 1.0)
        self.declare_parameter('smoothness_cost_gain', 0.1)

        # Goal position
        self.goal_x = 2.0
        self.goal_y = 2.0

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0

        # Laser data
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

        # Publishers/Subscribers
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # Timer at 10Hz
        self.timer = self.create_timer(0.1, self.dwa_control_loop)

        self.get_logger().info("✅ DWA Planner Initialized — Goal: ({}, {})".format(self.goal_x, self.goal_y))

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, self.theta = self.euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def dwa_control_loop(self):
        dw = self.dynamic_window()

        best_u = [0.0, 0.0]
        min_cost = float('inf')
        best_traj = None
        trajectories = []

        v_samples = np.arange(dw[0], dw[1], self.get_parameter('v_reso').value)
        w_samples = np.arange(dw[2], dw[3], self.get_parameter('yaw_rate_reso').value)

        for v in v_samples:
            for w in w_samples:
                traj = self.predict_trajectory(v, w)
                cost = self.calculate_trajectory_cost(traj, v, w)
                trajectories.append((traj, cost, v, w))

                if cost < min_cost:
                    min_cost = cost
                    best_u = [v, w]
                    best_traj = traj

        cmd = Twist()
        cmd.linear.x = best_u[0]
        cmd.angular.z = best_u[1]
        self.cmd_vel_pub.publish(cmd)

        self.visualize_trajectories(trajectories, best_traj)

        self.get_logger().info(f"🚀 Selected: v={best_u[0]:.2f} m/s, w={best_u[1]:.2f} rad/s | Cost: {min_cost:.2f}")

    def dynamic_window(self):
        vs = [
            self.get_parameter('min_speed').value,
            self.get_parameter('max_speed').value,
            -self.get_parameter('max_yaw_rate').value,
            self.get_parameter('max_yaw_rate').value
        ]

        vd = [
            self.v - self.get_parameter('max_accel').value * self.get_parameter('dt').value,
            self.v + self.get_parameter('max_accel').value * self.get_parameter('dt').value,
            self.w - self.get_parameter('max_dyaw_rate').value * self.get_parameter('dt').value,
            self.w + self.get_parameter('max_dyaw_rate').value * self.get_parameter('dt').value
        ]

        dw = [
            max(vs[0], vd[0]),
            min(vs[1], vd[1]),
            max(vs[2], vd[2]),
            min(vs[3], vd[3])
        ]
        return dw

    def predict_trajectory(self, v, w):
        traj = []
        x, y, theta = self.x, self.y, self.theta
        dt = self.get_parameter('dt').value
        predict_time = self.get_parameter('predict_time').value
        steps = int(predict_time / dt)

        for _ in range(steps):
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            traj.append([x, y, theta])

        return traj

    def calculate_trajectory_cost(self, traj, v, w):
        to_goal_cost = self.to_goal_cost(traj)
        obstacle_cost = self.obstacle_cost(traj)
        smoothness_cost = abs(v - self.v) + abs(w - self.w)

        total_cost = (
            self.get_parameter('to_goal_cost_gain').value * to_goal_cost +
            self.get_parameter('obstacle_cost_gain').value * obstacle_cost +
            self.get_parameter('smoothness_cost_gain').value * smoothness_cost
        )
        return total_cost

    def to_goal_cost(self, traj):
        if not traj:
            return float('inf')
        last_x, last_y, _ = traj[-1]
        return math.hypot(self.goal_x - last_x, self.goal_y - last_y)

    def obstacle_cost(self, traj):
        cost = 0.0
        min_obstacle_dist = 0.6  # danger threshold in meters

        for point in traj:
            x, y, _ = point
            dx = x - self.x
            dy = y - self.y
            angle_to_point = math.atan2(dy, dx) - self.theta

            # Normalize angle to [-pi, pi]
            while angle_to_point > math.pi:
                angle_to_point -= 2.0 * math.pi
            while angle_to_point < -math.pi:
                angle_to_point += 2.0 * math.pi

            index = int(round((angle_to_point - self.angle_min) / self.angle_increment))
            if 0 <= index < len(self.scan_ranges):
                r = self.scan_ranges[index]
                if 0.0 < r <= min_obstacle_dist:  # ← NOW INCLUDES EQUALITY
                    cost += 1.0 / r  # higher cost when closer
        return cost

    def visualize_trajectories(self, trajectories, best_traj):
        marker_array = MarkerArray()

        for i, (traj, cost, v, w) in enumerate(trajectories):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dwa_trajectories"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.color.a = 0.6

            # Color by cost: red (bad) → green (good)
            norm_cost = min(cost / 10.0, 1.0)
            marker.color.r = norm_cost
            marker.color.g = 1.0 - norm_cost
            marker.color.b = 0.0

            for point in traj:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
                marker.points.append(p)

            marker_array.markers.append(marker)

        # Highlight best trajectory
        if best_traj:
            best_marker = Marker()
            best_marker.header.frame_id = "odom"
            best_marker.header.stamp = self.get_clock().now().to_msg()
            best_marker.ns = "best_trajectory"
            best_marker.id = 9999
            best_marker.type = Marker.LINE_STRIP
            best_marker.action = Marker.ADD
            best_marker.scale.x = 0.05
            best_marker.color.a = 1.0
            best_marker.color.r = 0.0
            best_marker.color.g = 1.0
            best_marker.color.b = 0.0

            for point in best_traj:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
                best_marker.points.append(p)

            marker_array.markers.append(best_marker)

        self.traj_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
