#!/usr/bin/python
import rospy
import numpy as np 
from operator import itemgetter
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
from std_msgs.msg import Float64

class ConeDetector:
	def __init__(self):
		self.cone_sub = rospy.Subscriber("cone_location", Float32, self.phi_callback)
		self.cd_sub = rospy.Subscriber("scan", LaserScan, self.laser_callback)
		self.cd_pub = rospy.Publisher("cone_position", Point32, queue_size=4)
		self.phi = 1000.0
		self.phi_start=self.phi
		self.phi_end = self.phi
		self.window=3
	def phi_callback(self, msg):
		self.phi=msg.data
	def laser_callback(self,msg):
		if self.phi<np.pi:#check the angle
			phi_index=int((msg.angle_max+self.phi)/(msg.angle_max-msg.angle_min)*len(msg.ranges))
			points=msg.ranges[phi_index-self.window:phi_index+self.window]
			distance = np.mean(points)
			self.phi_start=self.phi-np.pi/(18+3*distance)
			self.phi_end=self.phi+np.pi/(18+3*distance)
			start_point=int((msg.angle_max+self.phi_start)/(msg.angle_max-msg.angle_min)*len(msg.ranges))
			end_point=int((msg.angle_max+self.phi_end)/(msg.angle_max-msg.angle_min)*len(msg.ranges))
			points=[]
			for i in range(start_point, end_point-5):
				wind=msg.ranges[i:i+6]
				mean=np.mean(wind)
				points.append((i+2,mean))
			point = min(points,key=lambda item:item[1])
			position = start_point+point[0]
			angle=msg.incr_angle*position+angle_min
			x=dist*np.cos(angle)
			y=dist*np.sin(angle)
			point = Point32()
			point.x=x
			point.y=y
			point.z=0.0
			self.cd_pub.publish(point)
		else:
			point=Point32()
			point.x=0.0
			point.y=0.0
			point.z=0.0
			self.cd_pub.publish(point)
if __name__=="__main__":
	rospy.init_node("ConeDetector")
	node=ConeDetector()
	rospy.spin()

