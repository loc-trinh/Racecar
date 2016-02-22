#!/usr/bin/python

###############
##
## Drive Control
##
## Takes in a distance to travel and an angle to turn, outputs a throttle and theta
##
###############

# Python Includes
import numpy as np

# ROS Includes
import rospy
import threading

# ROS messages
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

class DriveControl:
    def __init__(self):
        self.topic_theta = "wall_detector/theta"
        self.topic_obstacle="obstacle_distance"
        self.topic_output= "drive_control/ackermann_drive"
        self.max_steering_angle = 0.3
        self.k=1
        self.d0 = 1
        self.distance = 1
        self.lock = threading.Lock()

        #Pubs and Subs
        self.drive_pub = rospy.Publisher(self.topic_output, AckermannDriveStamped, queue_size=10)
        rospy.Subscriber(self.topic_theta, Float32, self.throttle_callback)
        rospy.Subscriber(self.topic_obstacle, Float32, self.obstacle_callback)

    def throttle_callback(self, data):
        with self.lock:
            d = self.distance

        k1 = 0.3
        k2 = 0.4

        theta = data.data

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        
        steering_d = (self.d0 - d)* k1 - theta*k2
        theta_parallel = max(min(steering_d,self.max_steering_angle), -1*self.max_steering_angle)

        msg.drive.steering_angle = theta_parallel
        msg.drive.speed = self.k 
        self.drive_pub.publish(msg)

    def distance_callback(self, data):
        with self.lock:
            self.distance = data.data;

    def obstacle_callback(self, data):
        if data.data <=2.0 and data.data >= 0:
            self.k=0
        else:
            self.k=1


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("drive_control")

    DriveControl();

    # enter the ROS main loop
    rospy.spin()

