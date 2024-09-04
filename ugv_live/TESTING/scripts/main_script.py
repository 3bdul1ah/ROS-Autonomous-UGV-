import rospy
from tf import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32, Quaternion
import math
import tf.transformations as tf_transformations

# Constants
WHEEL_RADIUS = 0.05
ENCODER_RESOLUTION = 725.0
WHEEL_BASE = 0.33

# Global Variables
prevRightEncoderTicks = 0.0
prevLeftEncoderTicks = 0.0
x_position = 0.0
y_position = 0.0
theta = 0.0

def encoder_callback(msg):
    global prevRightEncoderTicks, prevLeftEncoderTicks, x_position, y_position, theta
    right_ticks = float(msg.x)
    left_ticks = float(msg.y)

    delta_right = (2 * math.pi * WHEEL_RADIUS * (right_ticks - prevRightEncoderTicks)) / ENCODER_RESOLUTION
    delta_left = (2 * math.pi * WHEEL_RADIUS * (left_ticks - prevLeftEncoderTicks)) / ENCODER_RESOLUTION

    delta_distance = (delta_right + delta_left) / 2.0
    delta_theta = (delta_right - delta_left) / WHEEL_BASE

    x_position += delta_distance * math.cos(theta)
    y_position += delta_distance * math.sin(theta)
    theta += delta_theta

    prevRightEncoderTicks = right_ticks
    prevLeftEncoderTicks = left_ticks

def main():
    global prev_time
    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    trans_broadcaster = TransformBroadcaster()
    rospy.Subscriber('/encoder', Point32, encoder_callback)

    prev_time = rospy.Time.now()

    rate = rospy.Rate(10.0)  # 10 Hz

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Calculate the time elapsed
        dt = (current_time - prev_time).to_sec()

        # Compute velocity
        distance = ((2 * math.pi * WHEEL_RADIUS * (prevRightEncoderTicks - prevRightEncoderTicks)) / ENCODER_RESOLUTION +
                    (2 * math.pi * WHEEL_RADIUS * (prevLeftEncoderTicks - prevLeftEncoderTicks)) / ENCODER_RESOLUTION) / 2.0
        velocity_x = distance * math.cos(theta) / dt
        velocity_y = distance * math.sin(theta) / dt
        velocity_theta = (distance / dt) / WHEEL_BASE

        # Compute quaternion for the current theta
        odom_quat = tf_transformations.quaternion_from_euler(0, 0, theta)

        # Broadcast the transform
        trans_broadcaster.sendTransform(
            (x_position, y_position, 0.0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # Create and publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x_position
        odom.pose.pose.position.y = y_position
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.twist.twist.linear.x = velocity_x
        odom.twist.twist.linear.y = velocity_y
        odom.twist.twist.angular.z = velocity_theta

        odom_pub.publish(odom)

        prev_time = current_time
        rate.sleep()

if __name__ == '__main__':
    main()
