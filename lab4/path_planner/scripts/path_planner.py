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
        self.topic_position="cone_array"
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


        #Pubs and Subs
        self.drive_pub = rospy.Publisher(self.topic_output, Point, queue_size=10)
        rospy.Subscriber(self.topic_position, PoseArray, self.path_callback)

        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

    def path_callback(self, data):
        self.stampedpoint.header.stamp = self.listener.getLatestCommonTime(self.map_frame,data.header.frame_id)
        robot = self.listener.transformPoint(self.map_frame, self.stampedpoint)
        poses=data.poses
        cones=poses
        driveTo= Point()
        if 2*len(self.path) != len(cones):
            for cone in cones:
                if math.floor(cone.position.x) not in self.seen:
                    self.path.append((cone.position.x -.5, cone.position.y + self.side*0.5))
                    self.path.append((cone.position.x +0.5, cone.position.y + self.side*0.5))
                    self.side=self.side * -1
                    self.seen.append(math.floor(cone.position.x))
        if (robot.point.x >= self.path[self.nextPoint][0]):
            self.nextPoint+=1
        if (self.nextPoint==len(self.path)):
            driveTo.x=0
            driveTo.y=0
            #done, do not move
        else:
            print "------"
            print robot
            print self.path[self.nextPoint]
            x=self.path[self.nextPoint][0] - robot.point.x
            y=self.path[self.nextPoint][1] - robot.point.y
            # phi = math.atan2(y,x)
            # (r, p, yaw) = tf.transformations.euler_from_quaternion([robot.orientation.x, robot.orientation.y,robot.orientation.z, robot.orientation.w])
            # theta=phi-yaw
            # distance= math.pow(math.pow(x,2)+math.pow(y,2),0.5)
            # driveTo.x= math.cos(theta)*distance
            # driveTo.y= math.sin(theta)*distance
            sp = PointStamped()
            point = Point()
            point.x=x
            point.y=y
            sp.point=point
            sp.header.frame_id=self.map_frame
            sp.header.stamp = self.listener.getLatestCommonTime(self.base_frame,data.header.frame_id)

            driveTo = self.listener.transformPoint(self.base_frame, sp).point

            ## currently still in world frame, may need to rotate to 
        self.drive_pub.publish(driveTo)




        


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("path_planner")

    PathPlanner();

    # enter the ROS main loop
    rospy.spin()

