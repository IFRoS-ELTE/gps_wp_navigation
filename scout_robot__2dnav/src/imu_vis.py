#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Quaternion,Vector3Stamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2, degrees
import os
import numpy as np
class IMUOrientationVisualizer:
    def __init__(self):
        rospy.init_node("mag_calibration", anonymous=True)

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/imu/mag', Vector3Stamped, self.imu_mag_callback)

        self.mag_values = []

        #headings vars
        self.heading_tn = None
        self.rate = rospy.Rate(10)
 
        self.mag_heading = None
 
        # mag declination
        self.declination = 5.783333333
 
        self.seq_num = 0

        self.imu_pub = rospy.Publisher("imu_orntn", Marker, queue_size=10)
        self.imu_mag_pub = rospy.Publisher("mag_orntn", Marker, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.mag_values_acc = np.array([0, 0, 0]) #accumulate 15 imu/mag values

    def imu_mag_callback(self, msg):

        # Hard iron effect correction
        x = msg.vector.x + 0.1908 
        y = msg.vector.y + 0.0518
        z = msg.vector.z + 0.5559
        
        self.mag_values.append([x, y, z])

        heading_mag = -atan2(y, x) #rad
        heading_tru = heading_mag + np.deg2rad(self.declination) #rad

        #Normalize heading
        heading_tru = (heading_tru + 2 * math.pi) % (2 * math.pi) - math.pi/2
        self.heading_tn = heading_tru
        print("TN_HEADING: ", degrees(self.heading_tn))
        self.visualize_mag_heading()

    def rotate_to_east(self):
        if self.heading_tn is None:
            print("NO HEADING DATA")
            return
        
        heading_east = -math.pi / 2
        delta_heading = heading_east - self.heading_tn

        # Normalize d_heading
        delta_heading = (delta_heading + math.pi) % (2 * math.pi) - math.pi

        twist   = Twist()
        w       = 0.5 #rad/s

        if abs(delta_heading) > 0.05:
            twist.angular.z = w if delta_heading > 0 else -w 
            self.cmd_vel_pub.publish(twist)
        else:
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist) 
            print("ROBOT IS NOW FACING EAST!!! \nCLOSE THIS NODE")  

    #save imu/mag to text file
    def save_mag_value(self):
         np.savetxt(rospy.Time.now(), np.array(self.mag_values))

    def visualize_mag_heading(self):
        marker = Marker()
        marker.header.frame_id = "odom" 
        marker.header.stamp = rospy.Time.now()
        marker.ns = "imu_mag_orientation"
        marker.id = 1
        
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Convert heading (yaw) to quaternion
        quaternion = quaternion_from_euler(0, 0, self.heading_tn)
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
        # os.system('clear' if os.name == 'posix' else 'cls')

        quaternion = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        # print(f"Yaw_IMU: {yaw:.2f}({yaw*180/3.1415:.2f})")
        print("----------------------------")
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
        while not rospy.is_shutdown():
            self.rotate_to_east()
            self.rate.sleep()
            # rospy.spin()
if __name__ == '__main__':
    try:
        visualizer = IMUOrientationVisualizer()
        visualizer.run()
    except rospy.ROSInitException:
        pass