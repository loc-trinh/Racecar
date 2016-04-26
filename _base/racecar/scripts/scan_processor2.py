#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan

class HokuyoScanProcessor:
	def __init__(self):
		self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
		self.pub = rospy.Publisher("pscan", LaserScan, queue_size=0)

	def callback(self, msg):
		data = [0.0] * len(msg.intensities)
		for i in range(10,len(msg.intensities) - 10):
			if msg.intensities[i] < 0.1 and msg.intensities[i-2] < 0.1 and msg.intensities[i-1] < 0.1 and msg.intensities[i+1] < 0.1 and msg.intensities[i+2] < 0.1:
				data[i] = 100;
			else:
				data[i] = msg.intensities[i];

		msg.intensities = data;

		data = [0.0] * len(msg.ranges)
		for i in range(10,len(msg.ranges) - 10):
			if msg.ranges[i] > 30 and msg.ranges[i-2] > 30 and msg.ranges[i-1] > 30 and msg.ranges[i+1] > 30 and msg.ranges[i+2] > 30:
				data[i] = 29.0;
			else:
				data[i] = msg.ranges[i];

		msg.ranges = data;

		self.pub.publish(msg)


if __name__ == "__main__":
	rospy.init_node("dumbnode")
	node=HokuyoScanProcessor()
	rospy.spin()

