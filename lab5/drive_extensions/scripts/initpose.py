#!/usr/bin/python
import rospy
import roslib
import numpy as np
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

class TestStartNode:
    def __init__(self):
        self.pub = '/initialpose'
        self.drive_pub = rospy.Publisher(self.pub, PoseWithCovarianceStamped, queue_size=1)
        self.send_start()

    def send_start(self):
        print  "sending start"
        while not rospy.is_shutdown():
            start = PoseWithCovarianceStamped()
            start.pose.pose.position = Point(0.0, 0.0, 0.0)
            start.pose.pose.orientation.w = 0.0
            start.pose.covariance = np.diag(np.diag(np.arange(36).reshape((6,6))))
            start.header.frame_id = 'odom'
            start.header.stamp = rospy.Time.now()
            self.drive_pub.publish(start)

if __name__=="__main__":
    
    rospy.init_node('TestStartNode')
    node=TestStartNode()
    rospy.spin()
    
