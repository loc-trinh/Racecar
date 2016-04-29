#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import numpy as np 
class ObsticleOvoider:
	"""docstring for ObstacleOvoider"""
	def __init__(self):
		self.subs=rospy.Subscriber("/racecar/laser/scan", LaserScan, self.obsticleCallback)
		self.pubs=rospy.Publisher("escape_point", PointStamped, queue_size=3)
		self.front_wind=np.pi/36.0
		self.wind=5
		self.obs_threshold=1.0
	def means(self,window):
		"""
		-initializes the previous means if needed
		-calculates window means
		"""
		subwindows=int(len(window)/float(self.wind))
		if self.last_means==[]:
			self.last_means=np.ones(shape=(self.wind))*4
	
		new_window=window[0:subwindows*self.wind+1]
		means=np.mean(np.array(new_window).reshape(self.wind,subwindows),axis=1)
		return means
	def obsticleCallback(self,data):
		front_start=int((-self.front_wind+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		front_end=int((self.front_wind+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		print front_start,front_end
		wind=data.ranges[front_start:front_end]
		window=[]
		for i in range(len(wind)):
			if i<len(wind)-3:
				window.append(min(wind[i:i+2]))
			else:
				window.append(min(wind[i-2:i]))
