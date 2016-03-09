#!/usr/bin/python

###############
##
## Path planner
##
###############

# Python Includes
import numpy as np
import math

# ROS Includes
import rospy
import tf

# ROS messages
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped 


class PathPlanner:
    def __init__(self):
        self.topic_position="cone_position"
        self.topic_output= "drive_to_xy"
        self.side=-1
        self.path=[]
        self.seen=[]
        self.nextPoint=0

        self.map_frame = "odom"
        self.base_frame = "base_link"

        self.base_frame = rospy.get_param('~base_frame', self.base_frame)
        self.map_frame = rospy.get_param('~map_frame', self.map_frame)

        point=Point()
        point.x=0.0
        point.y=0.0
        point.z=0.0
        self.stampedpoint=PointStamped()
        self.stampedpoint.header.frame_id=self.base_frame
        self.stampedpoint.point=point

        self.robot=self.stampedpoint


        #Pubs and Subs
        self.drive_pub = rospy.Publisher(self.topic_output, Point, queue_size=10)
        rospy.Subscriber(self.topic_position, PointStamped, self.path_callback)

        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

    def publish(self):

        driveTo= Point()
        for node in self.path:
            print "------"
            x=node[0] - self.robot.point.x
            y=node[1] - self.robot.point.y
            
            sp = PointStamped()
            point = Point()
            point.x=x
            point.y=y
            sp.point=point
            sp.header.frame_id=self.map_frame
            sp.header.stamp = self.listener.getLatestCommonTime(self.base_frame,self.map_frame)

            driveTo = self.listener.transformPoint(self.base_frame, sp).point
            if driveTo.x>0:
                print driveTo
                break
            else:
                point= Point()
                point.x=1
                point.y=0
                driveTo=point

            ## currently still in world frame, may need to rotate to 
        self.drive_pub.publish(driveTo)

    def path_callback(self, data):
        self.stampedpoint.header.stamp = self.listener.getLatestCommonTime(self.map_frame,data.header.frame_id)
        self.robot = self.listener.transformPoint(self.map_frame, self.stampedpoint)
        cone=data

        data.header.stamp = self.listener.getLatestCommonTime(self.map_frame,data.header.frame_id)
        cone_loc = self.listener.transformPoint(self.map_frame, data)

        self.path=[]
        if cone.point.x > 0.5:
            self.path=[(cone_loc.point.x-0.5,cone_loc.point.y)]
        else:
            if len(self.path)==1:
                self.side= self.side*-1
            self.path=[(cone_loc.point.x, cone_loc.point.y+self.side*0.2), (cone_loc.point.x+0.5, cone_loc.point.y)]
        self.publish()
        




        


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("path_planner")

    pp=PathPlanner();

    while not rospy.is_shutdown():
        pp.publish()

    # enter the ROS main loop
    rospy.spin()

