#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import tf.transformations
import numpy as np
from math import atan2, asin, pi, degrees


def wrap_angle(angle):
    """this function wraps the angle between -pi and pi

    :param angle: the angle to be wrapped
    :type angle: float

    :return: the wrapped angle
    :rtype: float
    """
    return angle + (2.0 * np.pi * np.floor((np.pi - angle) / (2.0 * np.pi)))


def imu_callback(data):
    # Extract quaternion from IMU data
    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w,
    )

    # Convert quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # Create a message to publish Euler angles
    euler_msg = Float64MultiArray()
    euler_msg.data = euler
    angle = wrap_angle(euler[2])

    print("IMU : ", degrees(angle))
    # Publish the Euler angles
    euler_pub.publish(euler_msg)


def odom_callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    orientation = data.pose.pose.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

    angle = wrap_angle(euler[2])

    # print("Angle not filtered : ", degrees(angle))


def odom_callback_filter(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    orientation = data.pose.pose.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

    angle = wrap_angle(euler[2])

    # print("Filte : ", degrees(angle))


if __name__ == "__main__":
    rospy.init_node("imu_to_euler_node")
    # Subscriber to IMU data
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    # Publisher for Euler angles
    euler_pub = rospy.Publisher("/euler_angles", Float64MultiArray, queue_size=10)
    odom_filtered = rospy.Subscriber("/odometry/filtered", Odometry, odom_callback)
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback_filter)
    rospy.spin()
