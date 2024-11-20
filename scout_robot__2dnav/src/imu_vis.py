#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Quaternion,Vector3Stamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2, degrees
import os

class IMUOrientationVisualizer:
    def __init__(self):
        rospy.init_node("imu_orientation_visualizer", anonymous=True)

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/imu/mag', Vector3Stamped, self.imu_mag_callback)

        self.imu_pub = rospy.Publisher("imu_orientn", Marker, queue_size=10)
        self.imu_mag_pub = rospy.Publisher("imu_mag", Marker, queue_size=10)

        self.mag_heading = None

    def imu_mag_callback(self, msg):
        x = msg.vector.x
        y = msg.vector.y
        z = msg.vector.z

        heading = atan2(y, x)

        #* Low-pass filter the heading
        if self.mag_heading is None:
             self.mag_heading = heading
        else:
             self.mag_heading = 0.9 * self.mag_heading + 0.1 * heading

        heading_degrees = degrees(self.mag_heading)
        # rospy.loginfo(f"Magnetic Heading: {heading_degrees}")

        marker = Marker()
        marker.header.frame_id = "base_link" #! to see it move with the robot
        marker.header.stamp = rospy.Time.now()
        marker.ns = "imu_mag_orientation"
        marker.id = 1
        
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Convert heading (yaw) to quaternion
        quaternion = quaternion_from_euler(0, 0, -self.mag_heading)
        marker.pose.orientation = Quaternion(*quaternion)

        marker.scale.x = 2.1
        marker.scale.y = 2.1
        marker.scale.z = 0.1

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        start_point = Point(0, 0, 0)
        end_point = Point(1, 0, 0)

        marker.points = [start_point, end_point]

        self.imu_mag_pub.publish(marker)

    def imu_callback(self, msg):
        os.system('clear' if os.name == 'posix' else 'cls')

        quaternion = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        print(f"Yaw_m: {yaw:.2f}({yaw*180/3.1415:.2f})")
        #create marker msg
        marker = Marker()
        marker.header.frame_id = "base_link" #! to see it move with the robot
        marker.header.stamp = rospy.Time.now()
        marker.ns = "imu_orientation"
        marker.id = 0
        
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.orientation = quaternion

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.03

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        start_point = Point(0, 0, 0)
        end_point = Point(1, 0, 0)


        marker.points = [start_point, end_point]

        self.imu_pub.publish(marker)

    def run(self):
            rospy.spin()
if __name__ == '__main__':
    try:
        visualizer = IMUOrientationVisualizer()
        visualizer.run()
    except rospy.ROSInitException:
        pass