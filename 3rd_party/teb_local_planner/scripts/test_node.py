#!/usr/bin/python
import rospy
import roslib
# import actionlib
from geometry_msgs.msg import PoseStamped
# import move_base_msgs.msg

class TestDriveNode:
    def __init__(self):
        self.pub = '/move_base_simple/goal'
        self.drive_pub = rospy.Publisher(self.pub, PoseStamped, queue_size=1)
        self.send_goal()

    def send_goal(self):
        print  "sending goal"
        while not rospy.is_shutdown():
            goal = PoseStamped()
            goal.pose.position.x = 1.0
            goal.pose.position.y = 1.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            goal.header.frame_id = 'odom'
            goal.header.stamp = rospy.Time.now()
            self.drive_pub.publish(goal) 

if __name__=="__main__":
    rospy.init_node('TestDriveNode')
    node=TestDriveNode()
    rospy.spin()
