#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from tf.transformations import quaternion_from_euler
import tf
import math

# Robot parameters
ROBOT_WHEEL_RADIUS = 0.05  
ROBOT_WHEEL_BASE = 0.33 
ENCODER_RESOLUTION = 360 

# Initialize global variables
position_x = 0.0
position_y = 0.0
orientation_theta = 0.0
prev_right_encoder_ticks = 0
prev_left_encoder_ticks = 0
last_time = None 

def encoder_callback(data):
    global last_time, position_x, position_y, orientation_theta
    global prev_right_encoder_ticks, prev_left_encoder_ticks
    
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    
    # Retrieve encoder values
    new_right_encoder_ticks = data.x
    new_left_encoder_ticks = data.y
    
    # Compute distance traveled
    distance_right = (2 * math.pi * ROBOT_WHEEL_RADIUS * (new_right_encoder_ticks - prev_right_encoder_ticks)) / ENCODER_RESOLUTION
    distance_left = (2 * math.pi * ROBOT_WHEEL_RADIUS * (new_left_encoder_ticks - prev_left_encoder_ticks)) / ENCODER_RESOLUTION
    distance_center = (distance_right + distance_left) / 2
    
    # Update position and orientation
    position_x += distance_center * math.cos(orientation_theta)
    position_y += distance_center * math.sin(orientation_theta)
    orientation_theta += (distance_right - distance_left) / ROBOT_WHEEL_BASE
    
    # Compute velocities
    velocity_x = distance_center * math.cos(orientation_theta) / dt
    velocity_y = distance_center * math.sin(orientation_theta) / dt
    velocity_theta = (distance_right - distance_left) / ROBOT_WHEEL_BASE / dt

    # Create odometry message
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, orientation_theta)
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose.position.x = position_x
    odom.pose.pose.position.y = position_y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = odom_quat[0]
    odom.pose.pose.orientation.y = odom_quat[1]
    odom.pose.pose.orientation.z = odom_quat[2]
    odom.pose.pose.orientation.w = odom_quat[3]
    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = velocity_x
    odom.twist.twist.linear.y = velocity_y
    odom.twist.twist.angular.z = velocity_theta

    # Publish odometry data
    odom_publisher.publish(odom)
    
    last_time = current_time
    prev_right_encoder_ticks = new_right_encoder_ticks
    prev_left_encoder_ticks = new_left_encoder_ticks

def odometry_publisher_node():
    global last_time, odom_publisher
    
    rospy.init_node('odometry_publisher_node')
    
    # Initialize last_time after node initialization
    last_time = rospy.Time.now()

    rospy.Subscriber("/encoder_data", Point32, encoder_callback)
    odom_publisher = rospy.Publisher("odom", Odometry, queue_size=50)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        odometry_publisher_node()
    except rospy.ROSInterruptException:
        pass
