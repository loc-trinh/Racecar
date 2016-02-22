#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud

import threading


class WallDetectorNode:
	def __init__(self):
		self.x = []
		self.y = []

		self.lock = threading.Lock();
		# Subscribe to laser data
		rospy.Subscriber("point_cloud", PointCloud, self.point_cloud_callback)

		# Publish a distance to wall and signed angle to turn (radians)
		self.wall_pub_dist = rospy.Publisher("wall_detector/distance", Float32, queue_size=10)
		self.wall_pub_theta = rospy.Publisher("wall_detector/theta", Float32, queue_size=10)

	def point_cloud_callback(self, msg):
		with self.lock:
			for point in msg.points:
				#print point
				if point.x > -3 and point.x < 3 and point.y < -0.3 and point.y > -10:
					self.x.append(point.x)
					self.y.append(point.y)

		wall_msg_dist, wall_msg_theta = self.detect_wall()

		self.wall_pub_dist.publish(wall_msg_dist)
		self.wall_pub_theta.publish(wall_msg_theta)

		self.x = []
		self.y = []

	def detect_wall(self):
		"""
		Take point cloud from the published lidar node
		Publish distance to wall and theta (signed) to turn
		"""
		with self.lock:
			if(len(self.x) > 5):
				m, b = np.polyfit(self.x,self.y,1)
			else:
				return 1, 0;


		origin = [0,0]
		
		norm = [-m, 1]
		distance = abs(np.dot(norm, origin)-b)/np.linalg.norm(norm)
		angle = np.math.atan2(np.cross(norm, [0,1]), np.dot(norm, [0,1]))
		theta = np.math.pi/2 - angle if angle > 0 else -(np.math.pi/2 + angle)

		wall_msg_dist = Float32()
		wall_msg_dist.data = distance
		wall_msg_theta = Float32()
		wall_msg_theta.data = theta
		return wall_msg_dist, wall_msg_theta

if __name__ == "__main__":
	rospy.init_node("wall_detector_node")
	node = WallDetectorNode()
	rospy.spin()

