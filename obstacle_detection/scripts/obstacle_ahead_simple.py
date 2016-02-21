#!/usr/bin/python

###############
##
## Obstacle Ahead Simple
##
## Detects obstacles ahead, using pointcloud data as input
##
## Parameters:
##  - detection_zone = the width of the detection zone
##  - sensitivity = How sensitive we are to obstacles
##  - topic_pointcloud = Point cloud topic name
##  - topic_output = obstacle distance output topic
##
###############

# Python Includes
import numpy as np

# ROS Includes
import rospy

# ROS messages
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

class ObstacleDetector:
    def __init__(self):
        #Default settings
        self.detection_zone = 1;
        self.sensitivity = 1;
        self.topic_pointcloud = "pointcloud";
        self.topic_output = "obstacle_distance";

        #Load Settings
        self.detection_zone = rospy.get_param('~detection_zone', self.detection_zone)
        self.sensitivity = rospy.get_param('~sensitivity', self.sensitivity)
        self.topic_pointcloud = rospy.get_param('~topic_pointcloud', self.topic_pointcloud)
        self.topic_output = rospy.get_param('~topic_output', self.topic_output)

        #Pubs and Subs
        self.distance_pub = rospy.Publisher(self.topic_output, Float32, queue_size=10)
        rospy.Subscriber(self.topic_pointcloud, PointCloud, self.pointcloud_callback)


    def pointcloud_callback(self, data):
        distance = -1;

        filtered_points = []

        for point in data.points:
            if abs(point.y) < self.detection_zone and point.x < 10 and point.x > 0.1:
                filtered_points.append(point);

        distance = np.mean(filtered_points);

        self.distance_pub.publish(distance)


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("obstacle_ahead_simple")

    ObstacleDetector();

    # enter the ROS main loop
    rospy.spin()


