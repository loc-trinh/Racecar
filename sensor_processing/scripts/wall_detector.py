#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud

class WallDetectorNode:
	def __init__(self):
		# Subscribe to laser data
		rospy.Subscriber("point_cloud", PointCloud, self.point_cloud_callback)

		# Publish a distance to wall and signed angle(radians) to turn 
		self.wall_pub_dist = rospy.Publisher("wall_detector/distance", Float32, queue_size=10)
		self.wall_pub_theta = rospy.Publisher("wall_detector/theta", Float32, queue_size=10)

	def point_cloud_callback(self, msg):
		x = []
		y = []
		for point in msg.points:
			if -3 <= point.x <= 3 and -10 < point.y < -0.3:
				x.append(point.x)
				y.append(point.y)

		m, b = np.polyfit(x,y,1)

		origin = [0,0]
		norm = [-m, 1]

		distance = abs(np.dot(norm, origin)-b)/np.linalg.norm(norm)
		theta = -np.math.atan2(np.cross(norm, [0,1]), np.dot(norm, [0,1]))
		
		wall_msg_dist = Float32()
		wall_msg_theta = Float32()
		wall_msg_dist.data = distance
		wall_msg_theta.data = theta

		self.wall_pub_dist.publish(wall_msg_dist)
		self.wall_pub_theta.publish(wall_msg_theta)

if __name__ == "__main__":
	rospy.init_node("wall_detector_node")
	WallDetectorNode()
	rospy.spin()

