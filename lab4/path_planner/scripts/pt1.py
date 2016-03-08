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
import math

# ROS Includes
import rospy

# ROS messages
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped

class DriveControl:
    def __init__(self):
        self.topic_point="/cone_position"
        self.topic_output= "/vesc/ackermann_cmd_mux/input/nav"
        self.max_steering_angle = 0.3
        self.max_speed=0.5

        self.k=1
        self.kp=0.6*self.k
        self.ki=2*self.k
        self.kd=0.125*self.k
        self.lastDistance=0
        self.lastTheta=0
        self.distanceI=0
        self.thetaI=0

        #Pubs and Subs
        self.drive_pub = rospy.Publisher(self.topic_output, AckermannDriveStamped, queue_size=10)
        rospy.Subscriber(self.topic_point, PointStamped, self.drive_callback)

    def drive_callback(self, data):
        x= data.point.x
        y= data.point.y
        distance = math.pow(math.pow(x,2) + math.pow(y,2),  0.5)
        theta = math.atan2(y,x)

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        #Compute speed:
        if x > 0.5:
            msg.drive.speed= 0.5;
        else:
            msg.drive.speed= 0;


        #Backup if too close
        if abs(theta) > self.max_steering_angle:
            self.lastDistance=0
            self.lastTheta=0
            self.distanceI=0
            self.thetaI=0
            msg.drive.speed=-0.3
            msg.drive.steering_angle=0

        msg.drive.steering_angle = max(min(self.max_steering_angle,theta), -1*self.max_steering_angle)


        print (msg.drive.speed, msg.drive.steering_angle)
        self.drive_pub.publish(msg)

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("drive_control")

    DriveControl();

    # enter the ROS main loop
    rospy.spin()

