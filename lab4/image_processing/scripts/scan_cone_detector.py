#!/usr/bin/python
import rospy
import numpy as np 
from operator import itemgetter
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped 
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Float64

class ConeDetector:
	def __init__(self):
		self.cone_sub = rospy.Subscriber("cone_location", Float32, self.phi_callback)
		self.cd_sub = rospy.Subscriber("scan", LaserScan, self.laser_callback)
		self.cd_pub = rospy.Publisher("cone_position", PointStamped, queue_size=4)
		self.phi = 100.0
		self.phi_start=self.phi
		self.phi_end = self.phi
		self.window=3
		self.stampedpoint=PointStamped()
		self.counter=0
	def phi_callback(self, msg):
		#print "recieved phi"
		self.phi=msg.data
	def laser_callback(self,msg):
		#ang="resolution:%s"%str(msg.angle_max-msg.angle_min)
		time=rospy.Time.now()
		if self.phi<np.pi:#check the angle
			phi_index=int((msg.angle_max+self.phi)/(msg.angle_max-msg.angle_min)*len(msg.ranges))
			points=msg.ranges[phi_index-self.window:phi_index+self.window]
			distance = np.mean(points)
			self.phi_start=self.phi-np.pi/(18+3*distance)
			self.phi_end=self.phi+np.pi/(18+3*distance)
			start_point=int((msg.angle_max+self.phi_start)/(msg.angle_max-msg.angle_min)*len(msg.ranges))
			end_point=int((msg.angle_max+self.phi_end)/(msg.angle_max-msg.angle_min)*len(msg.ranges))
			#ang="start_point, end_point:%s"%str((start_point,end_point))
			#print ang
			#rospy.loginfo(ang)
			points=[]
			for i in range(start_point, end_point-5):
				wind=msg.ranges[i:i+6]
				mean=np.mean(wind)
				points.append((i+2,mean))
			point = min(points,key=lambda item:item[1])
			position = start_point+point[0]
			dist=point[1]
			angle=msg.angle_increment*position+msg.angle_min
			x=dist*np.cos(angle)
			y=dist*np.sin(angle)
			point = Point()
			point.x=x
			point.y=y
			point.z=0.0
			
			self.counter+=1
			self.stampedpoint.header.seq=self.counter
			self.stampedpoint.header.frame_id="base_link"
			self.stampedpoint.header.stamp=time
			rospy.loginfo("point: %s" % str(point))
			self.stampedpoint.point=point
			self.cd_pub.publish(self.stampedpoint)
		else:
			point=Point()
			point.x=0.0
			point.y=0.0
			point.z=0.0
			self.counter+=1
			self.stampedpoint.header.seq=self.counter
			self.stampedpoint.header.frame_id="base_link"
			self.stampedpoint.header.stamp=time
			self.stampedpoint.point=point
			self.cd_pub.publish(self.stampedpoint)
			
if __name__=="__main__":
	rospy.init_node("ConeDetector")
	node=ConeDetector()
	rospy.spin()

