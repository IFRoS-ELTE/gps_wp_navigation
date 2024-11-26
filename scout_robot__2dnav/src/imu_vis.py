#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Quaternion,Vector3Stamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2, degrees
import os
import numpy as np
import csv
class IMUOrientationVisualizer:
    def __init__(self):
        rospy.init_node("mag_calibration", anonymous=True)

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/imu/mag', Vector3Stamped, self.imu_mag_callback)

        self.imu_pub = rospy.Publisher("imu_orientn", Marker, queue_size=10)
        self.imu_mag_pub = rospy.Publisher("imu_mag", Marker, queue_size=10)

        self.mag_heading = None

        # mag declination
        self.declination = None #! for now..change later

        self.output_file = rospy.get_param("~output_file", "magnetometer_data.csv")
        # Open file for writing
        self.file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.file)

        # Write header
        self.csv_writer.writerow(['x', 'y', 'z'])

        self.mag_points = np.empty((0, 3), float)
    def imu_mag_callback(self, msg):

        ###? Personal observation: 
        #* The angle computed usign atan2 from the mag vector assumes clockwise rotation is positive
        #* However, the quaternion from the imu assumes counter-clockwise rotation is positive
        x = msg.vector.x
        y = msg.vector.y
        z = msg.vector.z

        self.mag_points = np.vstack((self.mag_points, [x, y, z]))
        self.csv_writer.writerow([x, y, z])
        #######################!After calibration####################################
        #* Matrix A for soft iron calibration
        A = np.empty((3, 3), float)

        #* vector b for soft iron calibration
        b = np.empty((1, 3), float)

        C = (np.array([x, y, z].reshape(-1, 1) - b)) @ A
        x = C[0], y = C[1], z = C[2]
        ###########################!###############################################
        heading = atan2(y, x) #assuming heading is in x axis
        heading_calib = -atan2(y, x) # to match the quaternion orientation

        #* Low-pass filter the heading
        if self.mag_heading is None:
             self.mag_heading = heading
        else:
             self.mag_heading = 0.9 * self.mag_heading + 0.1 * heading

        heading_deg = degrees(self.mag_heading)  + self.declination
        heading_deg_calib = degrees(heading_calib)  + self.declination
        
        print(f"lpass heading: {heading_deg:.2f}| calib heading: {heading_deg_calib:.2f}")

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

        marker.scale.x = 0.1
        marker.scale.y = 0.1
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
        marker.header.frame_id = "odom" #! to see it move with the robot
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