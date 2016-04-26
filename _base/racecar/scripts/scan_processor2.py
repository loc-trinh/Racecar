#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan

class HokuyoScanProcessor:
	def __init__(self):
		self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
		self.pub = rospy.Publisher("pscan", LaserScan, queue_size=0)

	def callback(self, msg):

		for intensity in msg.intensities:
			print intensity
			if intensity < 0.1:
				print "zero intensity found"
				intensity = 100;

		self.pub.publish(msg)


if __name__ == "__main__":
	rospy.init_node("dumb node")
	node=HokuyoScanProcessor()
	rospy.spin()

