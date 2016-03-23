#!/usr/bin/python
import rospy
import roslib
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped

class TestDriveNode:
    def __init__(self):
        self.pub = '/move_base_simple/goal'
        self.drive_pub = rospy.Publisher(self.pub, PoseWithCovarianceStamped, queue_size=1)

    def send_goal(self):
        print  "sending goal"
        while not rospy.is_shutdown():
            start = PoseWithCovarianceStamped()
            start.pose.position.x = 0.0
            start.pose.position.y = 0.0
            start.pose.position.z = 0.0
            start.pose.orientation.w = 0.0
            start.covariance = np.diag(np.diag(np.arange(36).reshape((6,6))))
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            self.drive_pub.publish(goal)

if __name__=="__main__":
    
    rospy.init_node('TestDriveNode')
    node=TestDriveNode()
    rospy.spin()
    
