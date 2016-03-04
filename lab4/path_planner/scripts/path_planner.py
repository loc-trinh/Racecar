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

class PathPlanner:
    def __init__(self):
        self.topic_position="cone_location"
        self.topic_output= "drive_to_xy"
        self.side=-1
        self.path=[]
        self.seen=[]
        self.nextPoint=0

        #Pubs and Subs
        self.drive_pub = rospy.Publisher(self.topic_output, Point, queue_size=10)
        rospy.Subscriber(self.topic_position, PoseArray, self.path_callback)

    def path_callback(self, data):
        poses=data.poses
        robot = poses[0]
        cones = poses[1:]
        driveTo= Point()
        if 2*len(self.path) != len(cones):
            for cone in cones:
                if robot.position.x < cone.position.x and cone.position.x not in self.seen:
                    self.path.append((cone.position.x -0.2, cone.position.y + self.side*0.2))
                    self.path.append((cone.position.x +0.2, cone.position.y + self.side*0.2))
                    self.side=self.side * -1
                    self.seen.append(cone.position.x)
        if (robot.position.x >= self.path[self.nextPoint][0]):
            self.nextPoint+=1
        if (self.nextPoint==len(self.path)):
            driveTo.x=0
            driveTo.y=0
            #done, do not move
        else:
            x=self.path[self.nextPoint][0] - robot.position.x
            y=self.path[self.nextPoint][1] - robot.position.y
            phi = math.atan2(y,x)
            (r, p, yaw) = tf.transformations.euler_from_quaternion([robot.orientation.x, robot.orientation.y,robot.orientation.z, robot.orientation.w])
            theta=phi-yaw
            distance= math.pow(math.pow(x,2)+math.pow(y,2),0.5)
            driveTo.x= math.cos(theta)*distance
            driveTo.y= math.sin(theta)*distance

            ## currently still in world frame, may need to rotate to 
        self.drive_pub.publish(driveTo)
        print self.path



        


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("path_planner")

    PathPlanner();

    # enter the ROS main loop
    rospy.spin()

