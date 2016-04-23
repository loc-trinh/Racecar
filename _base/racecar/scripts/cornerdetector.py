#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import numpy as np 
class CornerDetector:
	def __init__(self):
		self.subs=rospy.Subscriber("/racecar/laser/scan", LaserScan, self.cornercallback)
		self.bool=rospy.Publisher("corner/bool",Bool,queue_size=2)
		self.corner_angle=rospy.Publisher("corner/angle",Float32,queue_size=2)
		self.detection_window=[np.pi/6,np.pi/3] #where to watch
		self.k=40#number of sub windows
		self.turn=1#indicates which side to watch
		self.last_means=[]#means of the left window
		self.last_detection_state=False#wether something was detected for the last run
		self.change_threshold=3.0#change in the reading
	def means(self,window):
		"""
		-initializes the previous means if needed
		-calculates window means
		"""
		subwindows=int(len(window)/float(self.k))
		if self.last_means==[]:
			self.last_means=np.ones(shape=(self.k))*4
	
		new_window=window[0:subwindows*self.k+1]
		means=np.mean(np.array(new_window).reshape(self.k,subwindows),axis=1)
		return means
	def cornercallback(self,data):
		start=int((self.turn*self.detection_window[0]+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		end=int((self.turn*self.detection_window[1]+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))

		#print "start: ", start, "end: ",end

		window=data.ranges[start:end]
		if start>end:
			window=data.ranges[end:start]
		
		#print len(window)
		
		means=self.means(window)

		
		mean_difference=np.subtract(means,self.last_means)

		max_change=np.max(mean_difference)

	
		angle=0
		boool=False
		#print "angle"
		#print "differences: ", mean_difference
		if max_change>=self.change_threshold:
			print "means: ", means,"last_means: ", self.last_means
			boool=True
			index=(mean_difference).tolist().index(max_change)
			sub=int(len(window)/float(self.k))
			point = start+sub*index+sub/2.0
			if start>end:
				point = end+sub*index+sub/2.0
			print "point: ", point
			angle=point/(len(data.ranges))*(data.angle_max-data.angle_min)+data.angle_min
		


		#print "angle: ",angle,"bool: ",boool
		#if self.last_detection_state:
			
		#print "angle: ", angle, boool
		self.bool.publish(boool)
		self.corner_angle.publish(angle)

	
		self.last_detection_state=True
		if not boool:
			self.last_detection_state=False
		

		self.last_means=means
		
if __name__=="__main__":
	rospy.init_node("CornerDetector")
	node =CornerDetector()
	rospy.spin()



			








