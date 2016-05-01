#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import numpy as np 
class CornerDetector:
	def __init__(self):
		self.subs=rospy.Subscriber("/scan", LaserScan, self.cornercallback)
		self.bool=rospy.Publisher("corner/bool",Bool,queue_size=2)
		self.corner_angle=rospy.Publisher("corner/angle",Float32,queue_size=2)
		self.detection_window=[np.pi/6,np.pi/3] #where to watch
		self.k=5#number of sub windows
		self.turn=-1#indicates which side to watch
		self.last_means=[]#means of the left window
		self.last_detection_state=False#wether something was detected for the last run
		self.change_threshold=2.0#change in the reading
		self.wind=5
		self.angle_offset=np.pi/30
		self.front_wind=np.pi/144
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
	def corner_finder(self,start,end, data):
		windowr=data.ranges[start:end]
		if start>end:
			windowr=data.ranges[end:start]
		window=[]
		for i in range(len(windowr)):
			if i>=len(windowr)-2:
				window.append(min(windowr[i-2:i]))
			else:
				window.append(min(windowr[i:i+2]))

		print "end: start: ", start, end
		print len(window)
		angle=0
		boool=False
		maxChange=0
		index=0
		begin=0
		stop=begin<=end-2*self.wind-start
		if end<start:
			stop=begin<=start-2*self.wind-end

		while begin<=end-2*self.wind-start or begin<=start-2*self.wind-end:
			#print stop
			window1=window[begin:begin+self.wind]
			#print "window1: ",window1
			window2=window[begin+self.wind:begin+2*self.wind]
			#print "window2: ",window2
			if len(window1)==0 or len(window2)==0:
				print "empty slace:", begin
				print "window1,window2: ",window1,window2
			mean1=np.mean(window1)
			mean2=np.mean(window2)
			change=mean2-mean1
			if start<end:
				change=mean1-mean2
			begin+=1
			if change>maxChange:
				maxChange=change
				index=begin
		if maxChange>self.change_threshold:
			point=start+index
			angle=point/float(len(data.ranges))*(data.angle_max-data.angle_min)+data.angle_min
			if angle>0:
				angle=angle-self.angle_offset/2.0
			else:
				angle=angle+self.angle_offset
			boool=True
			print "point: ", point
		print "Angle: ", angle
		print "maxChange: ",maxChange
		return angle, boool
	def cornercallback(self,data):
		start=int((self.turn*self.detection_window[0]+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		end=int((self.turn*self.detection_window[1]+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))

		#print "start: ", start, "end: ",end

		
		
		#means=self.means(window)

		angle,boool=self.corner_finder(start,end,data)
		if not boool:
			start=int((-self.turn*self.detection_window[0]+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
			end=int((-self.turn*self.detection_window[1]+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
			front_start=int((-self.front_wind+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
			front_end=int((self.front_wind+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
			print np.mean(data.ranges[front_start:front_end]), "After No corner"
			if np.mean(data.ranges[front_start:front_end])<5.5:
				print "checking right"
				angle,boool=self.corner_finder(start,end,data)

		# mean_difference=np.subtract(means,self.last_means)

		#max_change=np.max(means)
		#print window
		####
		
		####
		
		#print "angle"
		#print "differences: ", mean_difference
		# if max_change>=self.change_threshold:
		# 	print "means: ", means,"last_means: ", self.last_means
		# 	boool=True
		# 	index=(means).tolist().index(max_change)
		# 	sub=int(len(window)/float(self.k))
		# 	point = start+sub*index+sub/2.0
		# 	if start>end:
		# 		point = end+sub*index+sub/2.0
		# 	print "point: ", point
		# 	angle=point/(len(data.ranges))*(data.angle_max-data.angle_min)+data.angle_min
		


		#print "angle: ",angle,"bool: ",boool
		#if self.last_detection_state:
			
		#print "angle: ", angle, boool
		self.bool.publish(boool)
		self.corner_angle.publish(angle)

	
		# self.last_detection_state=True
		# if not boool:
		# 	self.last_detection_state=False
		

		# self.last_means=means
		
if __name__=="__main__":
	rospy.init_node("CornerDetector")
	node =CornerDetector()
	rospy.spin()



			








