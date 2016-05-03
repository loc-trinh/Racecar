#!/usr/bin/python
import rospy
import roslib
import actionlib
from geometry_msgs.msg import PoseStamped
import move_base_msgs.msg

class TestDriveNode:
    def __init__(self):
        self.pub = '/move_base_simple/goal'
        self.drive_pub = rospy.Publisher(self.pub, MoveBaseGoal, queue_size=1)

    def send_goal(self):
        print  "sending goal"
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 1.0
        goal.target_pose.pose.position.z = 1.0
        goal.target_pose.pose.orientation.w = 1.0
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()
        drive_pub.publish(goal)

if __name__=="__main__":
    try:
        send_goal()
    except:
        pass
