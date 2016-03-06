#!/usr/bin/python

###############
##
## Cone Position Estimator
##
###############

# Python Includes
import numpy as np
import math

# ROS Includes
import rospy
import tf

# ROS messages
from geometry_msgs.msg import PoseArray, Pose, PointStamped

class ConeEstimator:
    def __init__(self):
        #Default settings
        self.kfactor = 0.2;
        self.topic_input = "cone_position";
        self.topic_output = "cone_array";
        self.base_frame = "base_link"
        self.map_frame = "odom";

        #Load Settings
        self.kfactor = rospy.get_param('~kfactor', self.kfactor)
        self.topic_input = rospy.get_param('~topic_input', self.topic_input)
        self.topic_output = rospy.get_param('~topic_output', self.topic_output)
        self.base_frame = rospy.get_param('~base_frame', self.base_frame)
        self.map_frame = rospy.get_param('~map_frame', self.map_frame)

        #Pubs and Subs
        self.publisher = rospy.Publisher(self.topic_output, PoseArray, queue_size=10)
        rospy.Subscriber(self.topic_input, PointStamped, self.estimator_callback)

        #Setup Data Structures
        self.cone_array = PoseArray();
        self.cone_array.header.frame_id = self.map_frame;

        rospy.loginfo("estimator node loaded")


    def estimator_callback(self, data):
        # Get point and transform into odom frame
        rospy.loginfo("estimator callback")

        data.header.stamp = listener.getLatestCommonTime(self.map_frame,data.header.frame_id)
        con_loc = listener.transformPose(self.map_frame, data)

        #compare against existing cones
        matched = False
        for cone in self.cone_array.poses:
            if sqrt( (con_loc.point.x - cone.position.x)**2 + (con_loc.point.y - cone.position.y)**2 ) < self.kfactor:
                cone.position.x = con_loc.point.x;
                cone.position.y = con_loc.point.y;
                break

        #No match, add to cone list
        if not matched:
            cone = Pose();
            cone.position.x = con_loc.point.x;
            cone.position.y = con_loc.point.y;
            self.cone_array.cones.append(cone);

        ## currently still in world frame
        self.cone_array.header.stamp = rospy.Time.now();
        self.self.publisher.publish(self.cone_array)



if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("cone_estimator")

    ConeEstimator();

    rospy.spin();
    
    # enter the ROS main loop
    '''
    rate = rospy.Rate(estimator.rate)
    while not rospy.is_shutdown():
        rospy.loginfo(hello_str)


        pub.publish(hello_str)
        rate.sleep()
    '''


