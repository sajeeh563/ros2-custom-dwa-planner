#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
import math

def euler_to_quaternion(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

class DummyRobot(Node):
    def __init__(self):
        super().__init__('dummy_robot')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        
        self.timer = self.create_timer(0.1, self.publish_odom)
        self.get_logger().info("Dummy Robot Initialized and simulating movement.")

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def publish_odom(self):
        dt = 0.1  # Corresponds to timer frequency
        
        # Update pose
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = euler_to_quaternion(self.theta, 0.0, 0.0)
        
        odom.twist.twist.linear = Vector3(x=self.v, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.w)
        
        self.odom_pub.publish(odom)
        self.get_logger().info(f"Publishing Odom: x={self.x:.2f}, y={self.y:.2f}, v={self.v:.2f}, w={self.w:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = DummyRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
