#!/usr/bin/python

import rospy
import numpy as np
import math
import tf

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray, Pose, PoseStamped


class GoToPointNode:    
    targetPose = None
    drive=False

    def __init__(self):
        # Default Settings
        self.k = 0.8
        self.topic_input = "/move_base_simple/goal"
        self.topic_output = "/vesc/ackermann_cmd_mux/input/nav"
        self.base_frame = "base_link"
        self.map_frame = "odom"

        # Initial Settings
        self.k_steer = 0.1
        self.kp = 0.6*self.k
        self.ki = 0
        self.kd = 0
        self.max_steering_angle = 0.2
        self.max_speed =.5
        self.lastDistance = 0
        self.lastTheta = 0
        self.distanceI = 0
        self.thetaI = 0
        self.threshold = 1
        self.rate=30

        # Param Settings
        self.kfactor = rospy.get_param('~k', self.k)
        self.topic_input = rospy.get_param('~topic_input', self.topic_input)
        self.topic_output = rospy.get_param('~topic_output', self.topic_output)
        self.base_frame = rospy.get_param('~base_frame', self.base_frame)
        self.map_frame = rospy.get_param('~map_frame', self.map_frame)
        self.max_speed = rospy.get_param('~max_vel', self.max_speed)
        self.max_steering_angle = rospy.get_param('~max_theta', self.max_steering_angle)
        self.threshold = rospy.get_param('~threshold', self.threshold)
        self.rate = rospy.get_param('~rate', self.rate)

        # Pubs and Subs
        self.drive_pub = rospy.Publisher(self.topic_output, AckermannDriveStamped, queue_size=1)
        rospy.Subscriber(self.topic_input, PoseStamped, self.new_dest_callback)

        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

    def doDrive(self):
        # Get point and transform base_frame
        if(self.drive):
            data.header.stamp = self.listener.getLatestCommonTime(self.base_frame,data.header.frame_id)
            dest = self.listener.transformPose(self.base_frame, data)
            
            x = dest.pose.position.x
            y = dest.pose.position.y
            
            # Computer dist and theta
            distance = math.pow(math.pow(x,2) + math.pow(y,2),  0.5)
            theta = math.atan2(y,x)
            rospy.loginfo("Distance: " + str(distance))

            if(distance < self.threshold):
                rospy.loginfo("Goal Reached!")
                self.drive = False
                return;


            # Prepare to Publish Drive Msg
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"

            # calc speed and theta
            # tested for points ahead/left, ahead/right, ahead, behind, and current loc
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
            self.drive_pub.publish(msg)


    def new_dest_callback(self,data):

        self.targetPose = data;
        self.drive = True

if __name__=="__main__":
    rospy.init_node("goto_pt")

    GotoPt = GoToPointNode()
    rate = rospy.Rate(GotoPt.rate)
    while not rospy.is_shutdown():
        GotoPt.doDrive();
        rate.sleep();
