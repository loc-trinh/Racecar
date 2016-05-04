#!/usr/bin/python

import rospy
import numpy as np
import math
import tf

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Bool

from nav_msgs.msg import Path


def distFromLine(x,y,x1,y1,x2,y2):
    numerator= abs(x*(y2-y1) - (y*(x2-x1)) +x2*y1 -y2*x1)
    denom = float(math.sqrt((y2-y1)**2 + (x2-x1)**2))
    return (numerator/denom)

def lineSign(x,y,x1,y1,x2,y2):
    slope = (y2-y1)/(x2-x1)
    b = y2-slope*x2
    yParallel= slope*x+b
    if yParallel > y:
        return 1
    else:
        return -1

class PIDControlNode:    
    targetPose = None
    drive = False
    reqed = False
    cPlan = None
    step = 0

    def __init__(self):


        self.topic_plan_in = "global_plan"


        # Param Settings
        self.topic_plan_in = rospy.get_param('~topic_plan_in', self.topic_plan_in)
        #self.rate = rospy.get_param('~rate', self.rate)

        # Pubs and Subs
        rospy.Subscriber(self.topic_plan_in, Path, self.new_plan_callback)

        # Default Settings
        self.k = 0.8
        self.topic_drive_out = "/vesc/ackermann_cmd_mux/input/nav"
        self.base_frame = "base_link"
        self.map_frame = "odom"

        # Initial Settings
        self.k_steer = 0.1
        self.kp = 0.6
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
        self.driveDamp=0.1

        # Param Settings
        self.k_steer = rospy.get_param('~k_steer', self.k_steer)
        self.kp = rospy.get_param('~k_dist', self.kp)
        self.topic_drive_out = rospy.get_param('~topic_drive_out', self.topic_drive_out)
        self.base_frame = rospy.get_param('~base_frame', self.base_frame)
        self.map_frame = rospy.get_param('~map_frame', self.map_frame)
        self.max_speed = rospy.get_param('~max_vel', self.max_speed)
        self.max_steering_angle = rospy.get_param('~max_theta', self.max_steering_angle)
        self.threshold = rospy.get_param('~threshold', self.threshold)
        self.rate = rospy.get_param('~rate', self.rate)

        # Pubs and Subs
        self.drive_pub = rospy.Publisher(self.topic_drive_out, AckermannDriveStamped, queue_size=1)

        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

    def doDrive(self):
        # Get point and transform base_frame

        if self.step>=2:
            self.targetPose = PoseStamped();
            self.targetPose.header = self.cPlan.header;
            self.targetPose.pose =  self.cPlan.poses[self.step].pose;

            self.prevPose = PoseStamped();
            self.prevPose.header = self.cPlan.header;
            self.prevPose.pose =self.cPlan.poses[self.step-1].pose;

            self.targetPose.header.stamp = self.listener.getLatestCommonTime(self.base_frame,self.targetPose.header.frame_id)
            target = self.listener.transformPose(self.base_frame, self.targetPose)

            self.prevPose.header.stamp = self.listener.getLatestCommonTime(self.base_frame,self.prevPose.header.frame_id)
            prev = self.listener.transformPose(self.base_frame, self.prevPose)
            
            xT = target.pose.position.x
            yT = target.pose.position.y

            xP= prev.pose.position.x
            yP = prev.pose.position.y

            xS=0
            yS=0 ##self
            
            # Computer dist and theta
            xdistance = math.pow(math.pow(xT-xS,2),  0.5) ## xdistance to target
            lineDist = distFromLine(xS,yS,xP,yP,xT,yT)
            dx = xT - xP
            dy = yT - yP
            rads = math.atan2(dy,dx)
            theta=rads
            #rospy.loginfo("Distance: " + str(distance))
            if(xdistance < self.threshold):
                #rospy.loginfo("Goal Reached!")
                self.step+=1


            # Prepare to Publish Drive Msg
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"


            msg.drive.speed= self.max_speed
            # calc speed and theta
            # tested for points ahead/left, ahead/right, ahead, behind, and current loc
            self.distanceI= self.distanceI+lineDist
            dp= self.kp * lineDist
            di= self.kd*(lineDist - self.lastDistance)
            dd= self.ki*(self.distanceI)
            self.lastDistance=lineDist
            

            self.thetaI=self.thetaI+theta
            tp= self.k_steer * theta
            ti= self.ki*(self.thetaI)
            td= self.kd* (theta-self.lastTheta)

            angle =( theta + self.driveDamp*(dp+di+dd) * lineSign(xS,yS,xP,yP,xT,yT))

            msg.drive.steering_angle= max(min(self.max_steering_angle,angle), -1*self.max_steering_angle)
            self.lastTheta=theta
            self.drive_pub.publish(msg)



    

    def new_plan_callback(self,data):
        #rospy.loginfo("New Plan Received")
        self.cPlan = data;
        self.planHeader=data.header
        self.step = 2;

if __name__=="__main__":
    rospy.init_node("pid_node")

    pid = PIDControlNode()
    rate = rospy.Rate(pid.rate)
    while not rospy.is_shutdown():
        pid.doDrive();
        rate.sleep();
