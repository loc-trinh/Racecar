#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

class WallDetectorNode:
	def __init__(self):
		self.x = []
		self.y = []

		# Subscribe to laser data
		rospy.Subscriber("ranges", Float32, self.ranges)

		# Publish a distance to wall and signed angle to turn (radians)
		self.wall_pub_dist = rospy.Publisher("wall_detector/distance", Float32)
		self.wall_pub_theta = rospy.Publisher("wall_detector/theta", Float32)

	def ranges(self, msg):
		for point in msg.points:
			self.x.append[point.x]
			self.y.append[point.y]

		self.detect_wall()

		self.wall_pub_dist.publish(wall_msg_dist)
		self.wall_pub_theta.publish(wall_msg_theta)

		self.x = []
		self.y = []

	def detect_wall(self):
		"""
		Take point cloud from the published lidar node
		Publish distance to wall and theta (signed) to turn
		"""

		# Loc's logic here

		wall_msg_dist = Float32()
		wall_msg_dist.data = # results here
		wall_msg_theta = Float32()
		wall_msg_theta.data = # results here

if __name__ == "__main__":
	rospy.init_node("wall_detector_node")
	node = WallDetectorNode()
	rospy.spin()

