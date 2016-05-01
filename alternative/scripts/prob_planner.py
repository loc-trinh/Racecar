#!/usr/bin/python
import rospy
import numpy as np 
import matplotlib.pyplot as plt 
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
from std_msgs.msg import Bool
class ProbNav:
	def __init__(self):
		self.subs=rospy.Subscriber("/racecar/laser/scan",LaserScan, self.goalcallback, queue_size=1)
		self.goal=rospy.Publisher("escape_point",Point32,queue_size=1)
		self.obs_detected=rospy.Publisher("obs_detected", Bool,queue_size=1)
		self.front_wind=np.pi/72
		self.window=np.pi/2.0
		self.mean_k=10
		self.memory=10
		self.obsticle_width=5
		self.front_thresh=3.5
		self.escape_thresh=4.5
	def dist_angle_to_xy_transform(self, index, dist, incr_angle, start_point, angle_min):
		position=start_point+index

		angle=incr_angle*position+angle_min

		x=dist*np.sin(angle)

		y=dist*np.cos(angle)

		return x,y 
	def goalcallback(self,data):
		#print "ange_pre",(self.window[1]+data.angle_max)/(data.angle_max-data.angle_min)
		start=int((-self.window+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		end=int((self.window+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		front_start=int((-self.front_wind+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		front_end=int((self.front_wind+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		mydata=[]
		means=[]
		point =Point32()
		print "start: ", front_start, "end: ",front_end
		for i in range(start,end):
			neighbors_mean=np.mean(data.ranges[i-self.mean_k:i+self.mean_k])
			means.append(neighbors_mean)
			mydata.append(np.mean(means))
			if len(means)>self.memory:
				means.pop(0)
		#print mydata
		front_window=data.ranges[front_start:front_end]
		index=0
		min_dist="inf"
		for i in range(len(front_window)):
			if i<len(front_window)-self.obsticle_width:
				m=np.mean(front_window[i:i+self.obsticle_width])
				if m<min_dist:
					min_dist=m 
					index=i
		obsticle_ahead=False
		if min_dist<=self.front_thresh:
			obsticle_ahead=True
			right=index+front_start
			left=index+front_start
			opening=False
			while not opening:
				right-=1
				left+=1
				rdist=np.mean(data.ranges[right-self.obsticle_width:right])
				ldist=np.mean(data.ranges[left:left+self.obsticle_width])
				if rdist>self.escape_thresh:
					r=right-2*self.obsticle_width
					opening=True
					x,y=self.dist_angle_to_xy_transform(r, data.ranges[r], data.angle_increment, start, data.angle_min)
					print "right_escape"
					print "x,y: index: ", x,y, r, opening
					point.x=x 
					point.y=y
					point.z=0
					self.goal.publish(point)
					self.obs_detected.publish(obsticle_ahead)
				elif ldist>self.escape_thresh:
					opening=True
					l=left+2*self.obsticle_width
					x,y=self.dist_angle_to_xy_transform(l, data.ranges[l], data.angle_increment, start, data.angle_min)
					print "left_escape"
					print "x,y: index: ", x,y, l,opening
					point.x=x 
					point.y=y
					point.z=0
					self.goal.publish(point)
					self.obs_detected.publish(obsticle_ahead)









		# xs=[]
		# ys=[]
		# for i in range(len(mydata)):
		# 	x,y=self.dist_angle_to_xy_transform(i, mydata[i], data.angle_increment, start, data.angle_min)
		# 	xs.append(x)
		# 	ys.append(y)
		# #print xs
		# plt.plot(xs,ys,'.')
		# plt.axis([-10,10,-10,10])
		# plt.show()
if __name__=="__main__":
	rospy.init_node("ProbNav")
	node=ProbNav()
	rospy.spin
