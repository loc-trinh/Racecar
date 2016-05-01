#!/usr/bin/python

import rospy
import numpy as np
import math
import tf

from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped

class PlanExecutor:    
    cPlan = None
    step = 0

    def __init__(self):
        # Default Settings
        self.topic_plan_in = "global_plan"
        self.topic_req = "plan_request"
        self.topic_output = "plan_executor/goal_out"
        #self.base_frame = "base_link"
        #self.map_frame = "odom"

        # Initial Settings
        #self.rate=30

        # Param Settings
        self.topic_plan_in = rospy.get_param('~topic_plan_in', self.topic_plan_in)
        self.topic_req = rospy.get_param('~topic_req_in', self.topic_req)
        self.topic_output = rospy.get_param('~topic_output', self.topic_output)
        #self.rate = rospy.get_param('~rate', self.rate)

        # Pubs and Subs
        self.pose_pub = rospy.Publisher(self.topic_output, PoseStamped, queue_size=1)
        rospy.Subscriber(self.topic_plan_in, Path, self.new_plan_callback)
        rospy.Subscriber(self.topic_req, Bool, self.req_callback)

        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

    def pubPlan(self):
        # Get point and transform base_frame
        if(step < len(self.cPlan.poses)):
            msg = PoseStamped();
            msg.header = cPlan.header;
            msg.pose = self.cPlan.poses[step].pose;
            self.pose_pub.publish(msg)

    def req_callback(self):
        self.step+=1;
        rospy.loginfo("Publishing Step " + str(self.step))
        self.pubPlan();

    def new_plan_callback(self,data):
        rospy.loginfo("New Plan Received")
        self.cPlan = data;
        self.step = 0;
        self.pubPlan();


if __name__=="__main__":
    rospy.init_node("plan_executor")

    plan_exec = PlanExecutor()
    rospy.spin();
