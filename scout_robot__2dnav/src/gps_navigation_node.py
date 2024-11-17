#!/usr/bin/env python3
import rospy
from gps_conversion import LatLonToCartesianConverter
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA, Float64MultiArray
from pynput import keyboard
import tf
import rospkg
import math
import os
from geometry_msgs.msg import PoseStamped
import numpy as np
import threading


class Gps_Navigation:

    def __init__(self):
        # initialize variables
        self.initialize_frame = True
        # Initilaize the state of the robot
        self.lock = threading.Lock()
        # gps and odom paths array
        self.gps_path = []
        self.odom_path = []
        self.waypoints_cart = []
        self.waypoints_gps = []
        self.dis_tolerance = 0.5
        self.count = 0  # waypoint counter
        # uncertainty of gps measurements in meters
        self.Rs = np.diag([0.00001, 0.00001])
        self.xk = np.array([0, 0, 0]).reshape(3, 1)
        self.P = np.eye(3) * 0.1
        # Get the current package path
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path(
            "scout_robot__2dnav"
        )  # Replace 'your_package_name' with the actual package name

        # Start listening for keyboard events
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        self.goal_reached = False
        # publishers
        # gps and odom path publishers
        self.gps_odom = rospy.Publisher("/gps_odom", Odometry, queue_size=10)
        self.gps_odom_filtered = rospy.Publisher(
            "/gps_odom/filtered", Odometry, queue_size=10
        )
        self.gps_path_pub = rospy.Publisher("/gps_path", MarkerArray, queue_size=10)
        self.odom_path_pub = rospy.Publisher("/odom_path", MarkerArray, queue_size=10)
        self.waypoint_viz = rospy.Publisher("/waypoint_viz", MarkerArray, queue_size=10)

        self.move_goal = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1
        )
        self.gps_filter_marker = rospy.Publisher(
            "/gps_filter_marker", Marker, queue_size=10
        )

        # subscribers
        self.gps_data_sub = rospy.Subscriber(
            "/navsat/fix", NavSatFix, self.gps_callback
        )
        self.gps_data_sub = rospy.Subscriber("/gnss", NavSatFix, self.gps_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # rospy.Subscriber("/imu/data", Imu, imu_callback)
        self.gps_rate_timer = rospy.Timer(rospy.Duration(5), self.gps_rate_callback)
        self.start_nav = False

        # self.navigate = rospy.Timer(rospy.Duration(1), self.waypoint_navigation)

    def gps_callback(self, data):

        self.latitude = data.latitude
        self.longitude = data.longitude
        self.altitude = data.altitude

        if self.initialize_frame:
            self.anchor_lat = self.latitude
            self.anchor_lon = self.longitude
            self.initialize_frame = False

            self.conv = LatLonToCartesianConverter(self.anchor_lat, self.anchor_lon)
            self.start_nav = True

        self.gps_x, self.gps_y = self.conv.ll_to_cartesian(
            self.latitude, self.longitude
        )

        gps_odom = Odometry()
        gps_odom.header.stamp = rospy.Time.now()
        gps_odom.header.frame_id = "odom"
        gps_odom.pose.pose.position.x = self.gps_x
        gps_odom.pose.pose.position.y = self.gps_y
        gps_odom.pose.pose.position.z = 0
        self.update_filter()

        self.gps_odom.publish(gps_odom)

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        self.orientation = data.pose.pose.orientation
        quaternion = (
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.linear_velocity = data.twist.twist.linear
        self.angular_velocity = data.twist.twist.angular
        self.xk = np.array([self.x, self.y, self.yaw]).reshape(3, 1)
        cov = data.pose.covariance

        self.P = np.array(
            [cov[0], cov[1], cov[5], cov[6], cov[7], cov[11], cov[30], cov[31], cov[35]]
        ).reshape(3, 3)

    def gps_rate_callback(self, event):
        self.gps_path.append((self.gps_x, self.gps_y))
        self.publish_gps_path()
        self.odom_path.append((self.x, self.y))
        self.publish_odom_path()

    def publish_gps_path(self):

        marker_line = MarkerArray()
        marker_line.markers = []

        path_list = []
        myMarker = Marker()

        for i, data in enumerate(self.gps_path):
            myMarker = Marker()
            myMarker.header.frame_id = "odom"
            myMarker.header.stamp = rospy.Time.now()
            myMarker.type = myMarker.SPHERE
            myMarker.action = myMarker.ADD
            myMarker.id = i
            myMarker.pose.orientation.x = 0.0
            myMarker.pose.orientation.y = 0.0
            myMarker.pose.orientation.z = 0.0
            myMarker.pose.orientation.w = 1.0
            myPoint = Point()
            myPoint.x = data[0]
            myPoint.y = data[1]
            myMarker.pose.position = myPoint
            myMarker.color = ColorRGBA(1, 0, 0, 1)
            myMarker.scale.x = 0.1
            myMarker.scale.y = 0.1
            myMarker.scale.z = 0.05
            path_list.append(myMarker)

        # print("gps_x", self.gps_x, "gps_y", self.gps_y)
        self.gps_path_pub.publish(path_list)

    def publish_odom_path(self):

        marker_line = MarkerArray()
        marker_line.markers = []

        path_list = []

        for i, data in enumerate(self.odom_path):
            myMarker = Marker()
            myMarker.header.frame_id = "odom"
            myMarker.header.stamp = rospy.Time.now()
            myMarker.type = myMarker.SPHERE
            myMarker.action = myMarker.ADD
            myMarker.id = i

            myMarker.pose.orientation.x = 0.0
            myMarker.pose.orientation.y = 0.0
            myMarker.pose.orientation.z = 0.0
            myMarker.pose.orientation.w = 1.0

            myPoint = Point()
            myPoint.x = data[0]
            myPoint.y = data[1]

            myMarker.pose.position = myPoint
            myMarker.color = ColorRGBA(0.0, 0, 1, 1)

            myMarker.scale.x = 0.1
            myMarker.scale.y = 0.1
            myMarker.scale.z = 0.05
            path_list.append(myMarker)
        # print("odom_x", self.x, "odom_y", self.y)
        self.odom_path_pub.publish(path_list)

    def on_press(self, key):
        try:
            if key.char == "~":
                self.save_gps_coordinates()
        except AttributeError:
            pass

    def save_gps_coordinates(self):
        # read the current package path
        dir_path = os.path.join(self.package_path, "media")
        # Check if the directory exists, if not, create it
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        # Construct the file path
        file_path = os.path.join(dir_path, "gps_coordinates.txt")
        with open(file_path, "a") as file:
            file.write(f"{self.latitude}, {self.longitude}\n")
        rospy.loginfo(f"Saved GPS coordinates: {self.latitude}, {self.longitude}")
        self.read_gps_points()

    def imu_callback(self, msg):
        orientation = msg.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        self.yaw = np.array([self.yaw]).reshape(1, 1)
        self.heading_update()

    def read_gps_points(self):
        file_path = self.package_path + "/media/gps_coordinates.txt"

        self.waypoints_gps = []
        self.waypoints_cart = []

        with open(file_path, "r") as file:
            data = file.readlines()
            for line in data:
                lat, lon = line.split(",")
                self.latitude = float(lat)
                self.longitude = float(lon)

                self.gps_x, self.gps_y = self.conv.ll_to_cartesian(
                    self.latitude, self.longitude
                )

                self.waypoints_gps.append((self.latitude, self.longitude))
                self.waypoints_cart.append((self.gps_x, self.gps_y))

        self.visualize_waypoints()

    # visualize waypoint
    def visualize_waypoints(self):
        marker_line = MarkerArray()
        marker_line.markers = []
        path_list = []
        myMarker = Marker()
        for i, data in enumerate(self.waypoints_cart):
            myMarker = Marker()
            myMarker.header.frame_id = "odom"
            myMarker.header.stamp = rospy.Time.now()
            myMarker.type = myMarker.SPHERE
            myMarker.action = myMarker.ADD
            myMarker.id = i
            myMarker.pose.orientation.x = 0.0
            myMarker.pose.orientation.y = 0.0
            myMarker.pose.orientation.z = 0.0
            myMarker.pose.orientation.w = 1.0
            myPoint = Point()
            myPoint.x = data[0]
            myPoint.y = data[1]
            myMarker.pose.position = myPoint
            myMarker.color = ColorRGBA(0.8, 0, 0, 1)
            myMarker.scale.x = 0.1
            myMarker.scale.y = 0.1
            myMarker.scale.z = 0.05
            path_list.append(myMarker)
        self.waypoint_viz.publish(path_list)

    # navigate to the next waypoint
    def waypoint_navigation(self, event):
        if self.waypoints_cart == [] and self.start_nav:
            self.read_gps_points()

        elif self.count < len(self.waypoints_cart) - 1:
            waypoint_length = len(self.waypoints_cart)
            final_goal = self.waypoints_cart[-1]
            for waypoint in self.waypoints_cart:
                # print("waypoint ", waypoint)
                goal = PoseStamped()
                x, y = waypoint
                self.goal_reached = False
                goal.header.frame_id = "odom"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = 0
                goal.pose.orientation.x = 0
                goal.pose.orientation.y = 0
                goal.pose.orientation.z = 0
                goal.pose.orientation.w = 1
                self.move_goal.publish(goal)
                while not self.goal_reached:

                    distance = math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)
                    if distance < self.dis_tolerance:
                        self.goal_reached = True
                        self.count += 1

    def heading_update(self):
        # Create a row vector of zeros of size 1 x 3*num_poses
        self.compass_Vk = np.diag([1])
        # define the covariance matrix of the compass
        self.compass_Rk = np.diag([0.01**2])
        # print("imu update")
        Hk = np.array([0, 0, 1])
        predicted_compass_meas = self.xk[-1]
        # Compute the kalman gain
        K = (
            self.Pk
            @ Hk.T
            @ np.linalg.inv(
                (Hk @ self.Pk @ Hk.T)
                + (self.compass_Vk @ self.compass_Rk @ self.compass_Vk.T)
            )
        )
        # Compute the innovation
        innovation = np.array(
            self.wrap_angle(self.yaw[0] - predicted_compass_meas)
        ).reshape(1, 1)

        I = np.eye(len(self.xk))
        with self.lock:
            self.xk = self.xk + K @ innovation
            self.Pk = (I - K @ Hk) @ self.Pk @ (I - K @ Hk).T

    def update_filter(self):
        # Kalman filter update
        Hk = np.array([[1, 0, 0], [0, 1, 0]])
        gps_measurement = np.array([self.gps_x, self.gps_y]).reshape(2, 1)
        K = self.P @ Hk.T @ np.linalg.inv(Hk @ self.P @ Hk.T + self.Rs)
        self.xk = self.xk + K @ (gps_measurement - Hk @ self.xk)
        self.P = (np.eye(3) - K @ Hk) @ self.P
        # Publish the filtered GPS data
        gps_odom_filtered = Odometry()
        gps_odom_filtered.header.stamp = rospy.Time.now()
        gps_odom_filtered.header.frame_id = "odom"
        gps_odom_filtered.pose.pose.position.x = self.xk[0]
        gps_odom_filtered.pose.pose.position.y = self.xk[1]
        gps_odom_filtered.pose.pose.position.z = 0
        covar = np.zeros(36)
        covar[0] = self.P[0, 0]
        covar[1] = self.P[0, 1]
        covar[5] = self.P[0, 2]
        covar[6] = self.P[1, 0]
        covar[7] = self.P[1, 1]
        covar[11] = self.P[1, 2]
        covar[30] = self.P[2, 0]
        covar[31] = self.P[2, 1]
        covar[35] = self.P[2, 2]
        gps_odom_filtered.pose.covariance = covar
        quat = tf.transformations.quaternion_from_euler(0, 0, self.xk[2])
        gps_odom_filtered.pose.pose.orientation = Quaternion(
            quat[0], quat[1], quat[2], quat[3]
        )
        self.gps_odom_filtered.publish(gps_odom_filtered)

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.ns = "gps_in_map"
        marker.id = np.random.randint(1000)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0)
        marker.pose.position.x = self.xk[0]
        marker.pose.position.y = self.xk[1]
        marker.pose.position.z = 0
        marker.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        self.gps_filter_marker.publish(marker)


if __name__ == "__main__":
    rospy.init_node("gps_navigation_node")
    gps_nav = Gps_Navigation()
    rospy.spin()
