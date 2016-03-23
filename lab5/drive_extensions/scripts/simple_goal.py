#!/usr/bin/python
import rospy
import roslib
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal

class TestDriveNode:
    def __init__(self):
        self.pub = '/move_base_simple/goal'
        self.drive_pub = rospy.Publisher(self.pub, MoveBaseGoal, queue_size=1)
        self.send_goal()
    def send_goal(self):
        print  "sending goal"
        while not rospy.is_shutdown():
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = 1.0
            goal.target_pose.pose.position.y = 1.0
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.w = np.pi/4
            #print goal.target_pose.pose.position.x
            goal.target_pose.header.frame_id = 'odom'

            goal.target_pose.header.stamp = rospy.Time.now()
            self.drive_pub.publish(goal)

if __name__=="__main__":
    
    rospy.init_node('TestDriveNode')
    node=TestDriveNode()
    
    rospy.spin()
    
