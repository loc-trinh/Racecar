#!/usr/bin/python
import rospy
import roslib
import move_base_msgs.msg

class TestDriveNode:
    def __init__(self):
    	self.pub = '/move_base_simple/goal'
        self.drive_pub = rospy.Publisher(self.pub, MoveBaseGoal, queue_size=1)

    def send_goal(self):
    	goal = MoveBaseGoal()
    	goal.target_pose.pose.position.x = 1.0
    	goal.target_pose.pose.position.y = 1.0
    	goal.target_pose.pose.orientation.w = 0.0
    	goal.target_pose.header.frame_id = 'testing'
    	goal.target_pose.header.stamp = rospy.Time.now()


if __name__=="__main__":
    try:
    	send_goal()
