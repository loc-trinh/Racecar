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
from geometry_msgs.msg import Point

class DriveControl:
    def __init__(self):
        self.topic_point="drive_to_xy"
        self.topic_output= "/vesc/ackermann_cmd_mux/input/nav"
        self.max_steering_angle = 0.2
        self.max_speed=.5

        self.k=0.8
        self.k_steer = 0.1
        self.kp=0.6*self.k
        self.ki=0
        self.kd=0
        self.lastDistance=0
        self.lastTheta=0
        self.distanceI=0
        self.thetaI=0

        #Pubs and Subs
        self.drive_pub = rospy.Publisher(self.topic_output, AckermannDriveStamped, queue_size=10)
        rospy.Subscriber(self.topic_point, Point, self.drive_callback)

    def drive_callback(self, data):
        x= data.x
        y= data.y
        distance = math.pow(math.pow(x,2) + math.pow(y,2),  0.5)
        theta = math.atan2(y,x)

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        if x<=0.2:
            msg.drive.steering_angle=theta
            msg.drive.speed=0.3

        

        else:
            self.distanceI= self.distanceI+distance
            dp= self.kp * distance
            di= self.kd*(distance - self.lastDistance)
            dd= self.ki*(self.distanceI)
            self.lastDistance=distance
            msg.drive.speed= max(0.1,min(self.max_speed, dp+di+dd))

            self.thetaI=self.thetaI+theta
            tp= self.k_steer * theta
            ti= self.ki*(self.thetaI)
            td= self.kd* (theta-self.lastTheta)
            msg.drive.steering_angle= max(min(self.max_steering_angle,theta), -1*self.max_steering_angle)
            self.lastTheta=theta
        print (msg.drive.speed, msg.drive.steering_angle)
        self.drive_pub.publish(msg)



        


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("drive_control")

    DriveControl();

    # enter the ROS main loop
    rospy.spin()

