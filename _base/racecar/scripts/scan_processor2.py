#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan

class HokuyoScanProcessor:
	def __init__(self):
		self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
		self.pub = rospy.Publisher("pscan", LaserScan, queue_size=0)

	def callback(self, msg):
		intensity = [0.0] * len(msg.intensities)
		r = [0.0] * len(msg.intensities)
		for i in range(0,len(msg.intensities)):
			if msg.intensities[i] < 0.1:

				#"flood fill"
				i_start = i;

				while msg.intensities[i] < 0.1 and i < len(msg.intensities)-1:
					i+=1;

				num_zeros = i-i_start
				

				if num_zeros > 25:
					#print num_zeros
					for j in range(i_start, i):
						intensity[j] = 100;
						r[j] = 29.0;
			else:
				intensity[i] = msg.intensities[i];
				r[i] = msg.ranges[i];

		msg.intensities = intensity;
		msg.ranges = r;

		self.pub.publish(msg)



if __name__ == "__main__":
	rospy.init_node("dumbnode")
	node=HokuyoScanProcessor()
	rospy.spin()

