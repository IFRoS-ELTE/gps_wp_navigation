#!/usr/bin/env python3
import rospy
from gps_conversion import LatLonToCartesianConverter
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from pynput import keyboard
import rospkg
import math
import os
from geometry_msgs.msg import PoseStamped


class Gps_Navigation:

    def __init__(self):
        # initialize variables
        self.initialize_frame = True
        # gps and odom paths array
        self.gps_path = []
        self.odom_path = []
        self.waypoints_cart = []
        self.waypoints_gps = []
        self.dis_tolerance = 0.5
        self.count = 0  # waypoint counter
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
        self.gps_path_pub = rospy.Publisher("/gps_path", MarkerArray, queue_size=10)
        self.odom_path_pub = rospy.Publisher("/odom_path", MarkerArray, queue_size=10)
        self.waypoint_viz = rospy.Publisher("/waypoint_viz", MarkerArray, queue_size=10)
        self.move_goal = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1
        )

        # subscribers
        self.gps_data_sub = rospy.Subscriber(
            "/navsat/fix", NavSatFix, self.gps_callback
        )
        self.odom_sub = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.odom_callback
        )

        self.gps_rate_timer = rospy.Timer(rospy.Duration(1), self.gps_rate_callback)
        self.start_nav = False
        self.navigate = rospy.Timer(rospy.Duration(1), self.waypoint_navigation)

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

        self.gps_odom.publish(gps_odom)

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        self.orientation = data.pose.pose.orientation
        self.linear_velocity = data.twist.twist.linear
        self.angular_velocity = data.twist.twist.angular

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
            myMarker.color = ColorRGBA(0.224, 1, 0, 1)
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
            myMarker.color = ColorRGBA(0.224, 0, 1, 1)

            myMarker.scale.x = 0.1
            myMarker.scale.y = 0.1
            myMarker.scale.z = 0.05
            path_list.append(myMarker)
        # print("odom_x", self.x, "odom_y", self.y)
        self.odom_path_pub.publish(path_list)

    def on_press(self, key):
        try:
            if key.char == "~":
                pass
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
            myMarker.color = ColorRGBA(0.224, 1, 0, 1)
            myMarker.scale.x = 0.1
            myMarker.scale.y = 0.1
            myMarker.scale.z = 0.05
            path_list.append(myMarker)
        self.waypoint_viz.publish(path_list)

    # navigate to the next waypoint
    def waypoint_navigation(self, event):
        if self.waypoints_cart == [] and self.start_nav:
            self.read_gps_points()
        waypoint_length = len(self.waypoints_cart)
        final_goal = self.waypoints_cart[-1]
        if self.count < waypoint_length - 1:
            for waypoint in self.waypoints_cart:
                print("waypoint ", waypoint)
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


if __name__ == "__main__":
    rospy.init_node("gps_navigation_node")
    gps_nav = Gps_Navigation()
    rospy.spin()
