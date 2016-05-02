#!/usr/bin/python

import rospy
import tf
import numpy as np 

import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped,PoseStamped

from std_msgs.msg import Bool 
class PointEstimator:
	"""docstring for PointEstimator"""
	def __init__(self):
		self.lslope=rospy.Subscriber("/walldata/lslope", Float32, self.lslopecallback)
		self.rslope=rospy.Subscriber("/walldata/rslope", Float32, self.rslopecallback)
		self.lintercept=rospy.Subscriber("/walldata/lintercept", Float32, self.l_y_callback)
		self.rintercept=rospy.Subscriber("/walldata/rintercept", Float32, self.r_y_callback)
		
		self.corner_angle=rospy.Subscriber("corner/angle",Float32, self.angle_callback)
		self.corner_detected=rospy.Subscriber("corner/bool", Bool, self.detection_callback)

		self.obs_ahead=rospy.Subscriber("obs_detected",Bool,self.obs_callback)
		self.escape=rospy.Subscriber("escape_point",Point32,self.escape_callback)


		self.pubs=rospy.Publisher("/move_base_simple/goal", PointStamped, queue_size=3)
		self.listener = tf.TransformListener(True, rospy.Duration(10.0))
		self.time = rospy.Time.now()
		#rospy.rate(10)

		self.left_slope=0
		self.right_slope=0
		self.lefty=0
		self.righty=0
		self.angle=0
		self.detected=False
		self.obs_detected=False
		self.escape=0

		self.dist=3.0
		self.publisher()
	def obs_callback(self,data):
		self.obs_detected=data.data
	def escape_callback(self,data):
		self.escape=data
		print "obsdata: ",data
	def lslopecallback(self, data):
		self.left_slope=data.data
	def rslopecallback(self,data):
		self.right_slope=data.data
	def l_y_callback(self,data):
		self.lefty=data.data
	def r_y_callback(self,data):
		self.righty=data.data
	def angle_callback(self,data):
		self.angle=data.data
	def detection_callback(self,data):
		self.detected=data.data
		# point =Point32()
		# spoint=PointStamped()
		# if not self.detected:
		# 	slope=(self.left_slope+self.right_slope)/2.0
			
		# 	x=self.dist*np.sqrt(1.0/(1+(slope)**2))
		# 	y=x*slope
		# 	if slope<0:
		# 		x=-self.dist*np.sqrt(1.0/(1+(slope)**2))
		# 		y=x*slope
		# 	point.x=x
		# 	point.y=y 
		# 	point.z=0.0
		# 	spoint.point = point 
		# 	spoint.header.frame_id="base_link"
		# 	spoint.header.stamp=self.time 
		# 	print "false: ", spoint
		# 	self.pubs.publish(spoint)
		# else:
		# 	x=self.dist*np.sin(angle)
		# 	y=self.dist*np.cos(angle)

		# 	point.x=x
		# 	point.y=y
		# 	point.z=0.0
		# 	spoint.point = point 
		# 	spoint.header.frame_id="base_link"
		# 	spoint.header.stamp=self.time 
		# 	print "True", point 
		# 	self.pubs.publish(spoint)
	def diff_dist(self,ar,al,br,bl,cl,cr,d, x):
		d1=(abs(ar*x-np.sqrt(d**2-x**2)+cr)/np.sqrt(ar**2+1))
		d2=(abs(al*x-np.sqrt(d**2-x**2)+cl)/np.sqrt(ar**2+1))
		#print "d1:",d1,"d2: ",d2
		return d1-1.0*d2
	def publisher(self):
		rate=rospy.Rate(10)
		corner_time=100
		while not rospy.is_shutdown():
			corner_time+=1
			if corner_time<100:
				continue 
			point =Point32()
			spoint=PointStamped()
			if not self.detected and not self.obs_detected:
				ar,al,cr,cl=self.right_slope,self.left_slope,self.righty,self.lefty
				br,bl=(ar)**2+1,(al)**2+1
				#g=br*cl**2-bl*cr**2 
				d=self.dist
				#a1,a2,b1,b2,c1,c2=ar,al,br,bl,cr,cl
				x=0.0
				current=abs(self.diff_dist(ar,al,br,bl,cl,cr,d, x))
				step=.005
				changing=True
				#for i in range(int(d*1/(2.0*step))):
				while changing:
					#print "Here"
					down=abs(self.diff_dist(ar,al,br,bl,cl,cr,d, x-step*(abs(x-step)<d)))
					up=abs(self.diff_dist(ar,al,br,bl,cl,cr,d, x+step*(x+step<d)))
					if min(down,up,current)==current:
						changing=False
					current=min(down,up,current)
					if current==down:
						x=x-step
					elif current==up:
						x=x+step 
				print "current: ", current, "x: ",x 

				xcpl=x
				ycpl=np.sqrt(d**2-x**2)
				slope=(self.left_slope+self.right_slope)/2.0
				right_dist=abs(cr)/np.sqrt(ar**2+1.0)
				left_dist=abs(cl)/np.sqrt(al**2+1.0)
				# if right_dist>left_dist:
				# 	xcpl=xcpl
				# else:
				# a=slope 
				# b=(self.lefty+self.righty)/2.0
				# x=3.0
				# y=(x-b)/(a+000000001)
				# 	xcpl=xcpl
				# y=-self.dist*np.sqrt(1.0/(1+(slope)**2))
				#x=y*slope
				# if slope<0:
				# 	xcpl=-xcpl
				# 	print "Negative Slope"
				# 	y=self.dist*np.sqrt(1.0/(1+(slope)**2))
				# 	x=y*slope
				point.x=abs(ycpl)
				point.y=-xcpl
				point.z=0.0
				spoint.point = point 
				#spoint.header.frame_id="odom"
				spoint.header.frame_id = 'base_link'
				spoint.header.stamp=self.time 
				print "false: ", spoint
				self.pubs.publish(spoint)
			elif self.detected:
				corner_time=0
				print "CornerDetected:"
				y=self.dist*np.sin(self.angle)
				x=self.dist*np.cos(self.angle)

				point.x=abs(x)
				point.y=-y
				point.z=0.0
				spoint.point = point 
				#spoint.header.frame_id="odom"
				spoint.header.frame_id = 'base_link'
				spoint.header.stamp=self.time 
				print "True", point 

				self.pubs.publish(spoint)
			else:
				print "OBS_DETECTED"
				point.x=max(4.0,abs(self.escape.x))
				point.y=self.escape.y 
				point.z=0.0
				spoint.point=point
				spoint.header.frame_id = 'base_link'
				spoint.header.stamp=self.time 
				self.pubs.publish(spoint)
			# goal = PoseStamped()
	  #       goal.pose.position.x = spoint.point.x
	  #       goal.pose.position.y = spoint.point.y
	  #       goal.pose.position.z = 0.0
	  #       goal.pose.orientation.w = 1.0#math.atan2(y,x)
	  #       goal.header.frame_id = 'base_link'
	  #       self.pubs.publish(goal)

			rate.sleep()
if __name__=="__main__":
	rospy.init_node("PointEstimator")
	node = PointEstimator()
	rospy.spin()


