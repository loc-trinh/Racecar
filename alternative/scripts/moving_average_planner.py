#!/usr/bin/python
import rospy
import numpy as np 
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
class MovingAveragePlanner:
	"""docstring for MovingAveragePlanner"""
	def __init__(self):
		plt.ion()
		self.subs=rospy.Subscriber("/racecar/laser/scan", LaserScan, self.goalCallback)
		self.goal=rospy.Publisher("/goal", Float32, queue_size=1)
		self.window=(np.pi/2,-pi/2)
		self.mean_k=5
		self.memory=5
	def dist_angle_to_xy_transform(self, index, dist, incr_angle, start_point, angle_min):
		position=start_point+index

		angle=incr_angle*position+angle_min

		x=dist*np.sin(angle)

		y=dist*np.cos(angle)

		return x,y 
	def goalCallback(self,data):
		print self.memory
		start=int((self.window[1]+data.angle_max)/(data.angle_max-data.angle_min))*len(data.ranges)
		end=int((self.window[0]+data.angle_max)/(data.angle_max-data.angle_min))*len(data.ranges)
		data=[]
		means=[]
		for i in range(start,end):
			neighbors_mean=np.mean(data.ranges[i-self.mean_k:i+self.mean_k])
			means.append(neighbors_mean)
			data.append(np.mean(means))
			if len(means)>self.memory:
				means.pop(0)
		xs=[]
		ys=[]
		for i in range(len(data)):
			x,y=self.dist_angle_to_xy_transform(i, data[i], data.angle_increment, start, data.angle_min)
			xs.append(x)
			ys.append(y)
		plt.plot(xs,ys,'.')
		plt.axis([-10,10,-10,10])
		plt.show()
if __name__ == "__main__":
	rospy.init_node("MovingAveragePlanner")
	node=MovingAveragePlanner()
	rospy.spin()




