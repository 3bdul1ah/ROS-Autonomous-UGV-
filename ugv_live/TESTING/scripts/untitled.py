#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32, Quaternion
import math

WHEEL_RADIUS = 0.257
TICKS_PER_REVOLUTION = 945.0
WHEEL_BASE = 1.2
MAX_VELOCITY = 0.4306073

UGV_Right_Wheel_Ticks_Old = 0.0
UGV_Right_Wheel_Ticks_New = 0.0
UGV_Left_Wheel_Ticks_Old = 0.0
UGV_Left_Wheel_Ticks_New = 0.0

UGV_Position_X = 0.0
UGV_Position_Y = 0.0
UGV_Orientation_Theta = 0.0

UGV_Linear_Velocity_X = 0.0
UGV_Linear_Velocity_Y = 0.0
UGV_Angular_Velocity_Theta = 0.0

UGV_Delta_Theta = 0.0

def encoder_callback(msg):
    global UGV_Right_Wheel_Ticks_Old, UGV_Right_Wheel_Ticks_New
    global UGV_Left_Wheel_Ticks_Old, UGV_Left_Wheel_Ticks_New
    global UGV_Position_X, UGV_Position_Y, UGV_Orientation_Theta
    global UGV_Delta_Theta

    UGV_Right_Wheel_Ticks_New = float(msg.x)
    UGV_Left_Wheel_Ticks_New = float(msg.y)

    distance_per_tick = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REVOLUTION
    UGV_Distance_Traveled = ((UGV_Right_Wheel_Ticks_New - UGV_Right_Wheel_Ticks_Old) * distance_per_tick +
                             (UGV_Left_Wheel_Ticks_New - UGV_Left_Wheel_Ticks_Old) * distance_per_tick) / 2

    UGV_Position_X += UGV_Distance_Traveled * math.cos(UGV_Orientation_Theta)
    UGV_Position_Y += UGV_Distance_Traveled * math.sin(UGV_Orientation_Theta)

    UGV_Delta_Theta = ((UGV_Right_Wheel_Ticks_New - UGV_Right_Wheel_Ticks_Old) * distance_per_tick -
                       (UGV_Left_Wheel_Ticks_New - UGV_Left_Wheel_Ticks_Old) * distance_per_tick) / WHEEL_BASE
    UGV_Orientation_Theta += UGV_Delta_Theta

    UGV_Right_Wheel_Ticks_Old = UGV_Right_Wheel_Ticks_New
    UGV_Left_Wheel_Ticks_Old = UGV_Left_Wheel_Ticks_New

def odometry_publisher():
    global UGV_Position_X, UGV_Position_Y, UGV_Orientation_Theta
    global UGV_Linear_Velocity_X, UGV_Linear_Velocity_Y, UGV_Angular_Velocity_Theta
    global UGV_Delta_Theta

    rospy.init_node('ugv_odometry_publisher')

    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
    tf_broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber("/encoder", Point32, encoder_callback)

    last_time = rospy.Time.now()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        distance_per_tick = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REVOLUTION
        UGV_Distance_Traveled = ((UGV_Right_Wheel_Ticks_New - UGV_Right_Wheel_Ticks_Old) * distance_per_tick +
                                 (UGV_Left_Wheel_Ticks_New - UGV_Left_Wheel_Ticks_Old) * distance_per_tick) / 2

        UGV_Linear_Velocity_X = UGV_Distance_Traveled * math.cos(UGV_Orientation_Theta) / dt
        UGV_Linear_Velocity_Y = UGV_Distance_Traveled * math.sin(UGV_Orientation_Theta) / dt
        UGV_Angular_Velocity_Theta = UGV_Delta_Theta / dt

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, UGV_Orientation_Theta)
        tf_broadcaster.sendTransform(
            (UGV_Position_X, UGV_Position_Y, 0.0),
            odom_quat,
            current_time,
            "link_base",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "link_base"

        odom.pose.pose.position.x = UGV_Position_X
        odom.pose.pose.position.y = UGV_Position_Y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        odom.twist.twist.linear.x = UGV_Linear_Velocity_X
        odom.twist.twist.linear.y = UGV_Linear_Velocity_Y
        odom.twist.twist.angular.z = UGV_Angular_Velocity_Theta

        odom_pub.publish(odom)

        last_time = current_time
        rate.sleep()

if __name__ == '__main__':
    try:
        odometry_publisher()
    except rospy.ROSInterruptException:
        pass
