#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point32, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import tf2_ros
import math


class OdomPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('odom_publisher', anonymous=True)

        # Set parameters
        self.wheel_base = rospy.get_param('~wheel_base', 0.35)  # Distance between wheels (meters)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.05)  # Radius of the wheel (meters)
        self.encoder_resolution = rospy.get_param('~encoder_resolution', 1024.0)  # Encoder resolution (ticks per revolution)

        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Initialize time and previous encoder readings
        self.last_time = rospy.Time.now()
        self.prev_right_encoder = None
        self.prev_left_encoder = None

        # Set up publishers and subscribers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.encoder_sub = rospy.Subscriber('/encoder_data', Point32, self.encoder_callback)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()

    def encoder_callback(self, encoder_msg):
        current_time = rospy.Time.now()

        # Get current encoder positions
        right_encoder = encoder_msg.x
        left_encoder = encoder_msg.y

        # Skip the first callback to set the initial encoder values
        if self.prev_right_encoder is None or self.prev_left_encoder is None:
            self.prev_right_encoder = right_encoder
            self.prev_left_encoder = left_encoder
            return

        # Calculate the change in encoder positions
        delta_right = (right_encoder - self.prev_right_encoder) * (2 * math.pi * self.wheel_radius) / self.encoder_resolution
        delta_left = (left_encoder - self.prev_left_encoder) * (2 * math.pi * self.wheel_radius) / self.encoder_resolution

        # Update previous encoder values
        self.prev_right_encoder = right_encoder
        self.prev_left_encoder = left_encoder

        # Calculate distance traveled and change in orientation
        delta_s = (delta_right + delta_left) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_base

        # Calculate changes in x and y positions
        delta_x = delta_s * math.cos(self.theta + delta_theta / 2.0)
        delta_y = delta_s * math.sin(self.theta + delta_theta / 2.0)

        # Update robot's position and orientation
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Calculate velocities
        dt = (current_time - self.last_time).to_sec()
        vx = delta_s / dt
        vth = delta_theta / dt

        # Create and populate the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom_quat = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # Set the velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth

        # Publish the Odometry message
        self.odom_pub.publish(odom)

        # Broadcast the transform over TF
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        self.odom_broadcaster.sendTransform(odom_trans)

        # Update the last time
        self.last_time = current_time

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        odom_publisher = OdomPublisher()
        odom_publisher.run()
    except rospy.ROSInterruptException:
        pass

# ------------
