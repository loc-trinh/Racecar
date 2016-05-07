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

		self.lslope=rospy.Subscriber("/walldata", Point32, self.wallcallback)

		
		self.corner_angle=rospy.Subscriber("corner/angle",Float32, self.angle_callback)
		self.corner_detected=rospy.Subscriber("corner/bool", Bool, self.detection_callback)

		self.obs_ahead=rospy.Subscriber("obs_detected",Bool,self.obs_callback)
		self.escape=rospy.Subscriber("escape_point",Point32,self.escape_callback)


		#self.pubs=rospy.Publisher("/point_position", PointStamped, queue_size=3)
		self.pubs=rospy.Publisher("/plan_executor/goal_out", PoseStamped, queue_size=3)
		self.listener = tf.TransformListener(True, rospy.Duration(10.0))
		self.time = rospy.Time.now()
		#rospy.rate(10)

		self.walldata_y=0.0
		self.walldata_x=0.0
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
	def wallcallback(self, data):
		self.walldata_x=data.x 
		self.walldata_y=data.y
	def angle_callback(self,data):
		self.angle=data.data
	def detection_callback(self,data):
		self.detected=data.data
		
	
	def publisher(self):
		rate=rospy.Rate(60)
		corner_time=100
		while not rospy.is_shutdown():
			corner_time+=1
			if corner_time<100:
                                print "pass"
				continue 
			point =Point32()
			spoint=PointStamped()
			if not self.detected and not self.obs_detected:
				
				point.x=self.walldata_x
				point.y=self.walldata_y
				point.z=0.0
				spoint.point = point 
				#spoint.header.frame_id="odom"
				spoint.header.frame_id = 'base_link'
				spoint.header.stamp=self.time 
				print "false: ", spoint
				#self.pubs.publish(spoint)
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

				#self.pubs.publish(spoint)
			else:
				print "OBS_DETECTED"
				point.x=max(4.0,abs(self.escape.x))
				point.y=self.escape.y 
				point.z=0.0
				spoint.point=point
				spoint.header.frame_id = 'base_link'
				spoint.header.stamp=self.time 

				#self.pubs.publis
			goal = PoseStamped()
		        goal.pose.position.x = spoint.point.x
			goal.pose.position.y = -spoint.point.y
	        	goal.pose.position.z = 0.0
		        goal.pose.orientation.w = 1.0#math.atan2(y,x)
		        goal.header.frame_id = 'base_link'
		        self.pubs.publish(goal)
			rate.sleep()
if __name__=="__main__":
	rospy.init_node("PointEstimator")
	node = PointEstimator()
	rospy.spin()


