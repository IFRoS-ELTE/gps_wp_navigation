#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import os

class PointsVisualizer:
    def __init__(self):
        rospy.init_node("points_visualizer", anonymous=True)
        self.marker_pub = rospy.Publisher("gps_cart", MarkerArray, queue_size=10)
        self.file_path = os.path.join("/home/zed/zed-ws/src/lidar_clip/gps_carts", 'xy.txt')
        self.points = self.read_points_from_file(self.file_path)
        # self.publish_points()

        timer = rospy.Timer(rospy.Duration(1), self.publish_points)

    def read_points_from_file(self, file_path):
        points = []
        with open(file_path, 'r') as file:
            for line in file:
                x, y = map(float, line.strip().split(','))
                points.append((x, y))
        return points

    def publish_points(self, event):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x, y, 0)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.5
            marker.scale.y = 1.5
            marker.scale.z = 1.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        print("points published")
        print(f"Number of points: {len(self.points)}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        visualizer = PointsVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass