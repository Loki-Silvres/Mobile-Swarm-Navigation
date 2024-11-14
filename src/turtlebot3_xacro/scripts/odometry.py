#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from math import cos, sin


class Wheel:
    rad = 0.033
    separation = 0.287
    vmax = 0.26

    @staticmethod
    def scale(v, w):
        vl = abs(v + Wheel.separation*w/2)
        vr = abs(v - Wheel.separation*w/2)
        scale = max(vl,vr)/Wheel.vmax

        if scale > 1:
            return v/scale, w/scale
        return v, w


def to_sec(t):
    return t.sec + t.nanosec*1e-9


class TurtleOdometry(Node):

    def __init__(self):
        super().__init__('odometry')
        Wheel.rad = self.declare_parameter("wheels.radius", 0.033).value
        Wheel.separation = self.declare_parameter("wheels.separation", 0.287).value
        Wheel.vmax = self.declare_parameter("wheels.max_vel", 0.26).value

        pub_tf = self.declare_parameter("publish_tf", True).value

        self.odom = Odometry()
        self.theta = 0.

        self.odom.header.frame_id = self.declare_parameter("odom.frame_id", "waffle1/odom").value
        self.odom.child_frame_id = self.declare_parameter("odom.child_frame_id", "waffle1/base_footprint").value

        if pub_tf:
            self.br = TransformBroadcaster(self)
            self.tf = TransformStamped()
            self.tf.header.frame_id = self.odom.header.frame_id
            self.tf.child_frame_id = self.odom.child_frame_id
            self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        else:
            self.br = None
            # probably some EKF
            self.odom_pub = self.create_publisher(Odometry, "odom_raw", 10)

        self.js_sub = self.create_subscription(JointState, "joint_states", self.update, 10)
        self.t_prev = None
        self.angle_prev = [0., 0.]

        self.odom.pose.covariance[0] = 0.05
        self.odom.pose.covariance[7] = 0.05
        self.odom.pose.covariance[14] = 1.0e-9
        self.odom.pose.covariance[21] = 1.0e-9
        self.odom.pose.covariance[28] = 1.0e-9
        self.odom.pose.covariance[35] = 0.0872665

        self.odom.twist.covariance[0] = 0.01
        self.odom.twist.covariance[7] = 1.0e-9
        self.odom.twist.covariance[14] = 1.0e-9
        self.odom.twist.covariance[21] = 1.0e-9
        self.odom.twist.covariance[28] = 1.0e-9
        self.odom.twist.covariance[35] = 0.01

        if Wheel.vmax > 0:
            self.cmb_pub = self.create_publisher(Twist, 'cmd_vel_scaled', 1)
            self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.scale, 1)

    def scale(self, v: Twist):
        scaled = v
        scaled.linear.x, scaled.angular.z = Wheel.scale(scaled.linear.x, scaled.angular.z)
        self.cmb_pub.publish(scaled)

    def update(self, js: JointState):
        dt, wl, wr = self.extractJointVelocity(js)

        if dt is None:
            return

        v = self.odom.twist.twist.linear.x = .5*(wl+wr)*Wheel.rad
        w = self.odom.twist.twist.angular.z = (wr-wl)*Wheel.rad/Wheel.separation

        # update pose
        self.theta += w*dt/2
        self.odom.pose.pose.position.x += v*cos(self.theta)*dt
        self.odom.pose.pose.position.y += v*sin(self.theta)*dt
        self.theta += w*dt/2

        self.odom.pose.pose.orientation.z = sin(self.theta/2)
        self.odom.pose.pose.orientation.w = cos(self.theta/2)
        self.odom.header.stamp = js.header.stamp

        self.odom_pub.publish(self.odom)

        if self.br is not None:
            self.tf.transform.translation.x = self.odom.pose.pose.position.x
            self.tf.transform.translation.y = self.odom.pose.pose.position.y
            self.tf.transform.rotation.z = self.odom.pose.pose.orientation.z
            self.tf.transform.rotation.w = self.odom.pose.pose.orientation.w
            self.tf.header.stamp = js.header.stamp

            self.br.sendTransform(self.tf)

    def extractJointVelocity(self, js: JointState):
        left_idx = 0 if js.name[0] == 'wheel_left_joint' else 1
        right_idx = 1-left_idx
        t = to_sec(js.header.stamp)

        if self.t_prev is not None and t != self.t_prev:
            dt = t - self.t_prev
            wl = (js.position[left_idx] - self.angle_prev[0])/dt
            wr = (js.position[right_idx] - self.angle_prev[1])/dt
        else:
            dt = wl = wr = None

        self.angle_prev[0] = js.position[left_idx]
        self.angle_prev[1] = js.position[right_idx]
        self.t_prev = t

        return dt, wl, wr


if __name__ == '__main__':
    rclpy.init()
    robot = TurtleOdometry()
    rclpy.spin(robot)

