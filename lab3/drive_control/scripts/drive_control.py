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
        self.topic_point="cone_location"
        self.topic_output= "/vesc/ackermann_cmd_mux/input/teleop"
        self.max_steering_angle = 0.3
        self.max_speed=4

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
        rospy.Subscriber(self.topic_point, Point, self.drive_callback)

    def drive_callback(self, data):
        x= data.x
        y= data.y
        distance = math.pow(math.pow(x,2) + math.pow(y,2),  0.5)-0.25 # want to stop slightly in front of the cone
        theta = math.atan2(y,x)

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        if abs(theta) > self.max_steering_angle:
            self.lastDistance=0
            self.lastTheta=0
            self.distanceI=0
            self.thetaI=0
            msg.drive.speed=-0.5
            msg.drive.steering_angle=0

        else:
            self.distanceI= self.distanceI+distance
            dp= self.kp * distance
            di= self.kd*(distance - self.lastDistance)
            dd= self.ki*(self.distanceI)
            self.lastDistance=distance
            msg.drive.speed= min(self.max_speed, dp+di+dd)

            self.thetaI=self.thetaI+theta
            tp= self.kp * theta
            ti= self.ki*(self.thetaI)
            td= self.kd* (theta-self.lastTheta)
            msg.drive.steering_angle= max(min(self.max_steering_angle,tp+ti+td), -1*self.max_steering_angle)
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

