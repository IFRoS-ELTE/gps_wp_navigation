#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import Imu
import math

class MagCalibration:
    def __init__(self):
        rospy.init_node("mag_calibration", anonymous=True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #parameters
        self.vx = 1#m/s
        self.a_amp = 0.5 #rad/s
        self.l_d = 10 #sec

        self.start_time = rospy.Time.now()

        self.vel_msg = Twist()

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while  not rospy.is_shutdown():
            dt = (rospy.Time.now() - self.start_time).to_sec()

            #phase
            t = (2*math.pi * dt) / self.l_d
            wx = self.a_amp * math.sin(t)

            #velocity
            self.vel_msg.linear.x = self.vx
            self.vel_msg.angular.z = wx

            #publish the velocity
            self.velocity_publisher.publish(self.vel_msg)

            rate.sleep()

    def stop_robot(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        rospy.loginfo("Robot stopped")

    
if __name__ == '__main__':
    try:
        calibration = MagCalibration()
        calibration.run()
    except rospy.ROSInterruptException:
        pass

