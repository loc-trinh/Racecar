#!/usr/bin/python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class HokuyoScanProcessor:
	def __init__(self):
		

		self.hsp_subs=rospy.Subscriber("scan", LaserScan, self.hsp_callback)

		#self.range_pub=rospy.Publisher("ranges", Float32, queue_size=10)
		self.point_cloud=rospy.Publisher("point_cloud", PointCloud,queue_size=10)
		
		#choose the range you want
		self.min_angle=-np.pi/2 
		self.max_angle=np.pi/2

		self.deviation_threshold=2.0
		self.test_window=20
		plt.ion()
		#test if a point is an outlier
	def outlier_test(self, index, ranges):

		data_point=ranges[index]
		test=[]
		if index<self.test_window:

			test=ranges[0:index+self.test_window]

		elif index+self.test_window>len(ranges):
			test=ranges[index-int(self.test_window/2.0):]

		else:
			test=ranges[index-self.test_window:index+self.test_window]

		mean=np.mean(test)
		std_dev=np.std(test)

		index_deviation=abs(ranges[index]-mean)

		if index_deviation>self.deviation_threshold+std_dev:
			if ranges[index]>mean:
				data_point=mean+std_dev
			else:
				data_point=mean-std_dev
		return data_point





	def filter(self, ranges):
		my_ranges=[0]*len(ranges)
		for i in range(len(ranges)):
			my_ranges[i]=self.outlier_test(i, ranges)
		return my_ranges

	def dist_angle_to_xy_transform(self, index, dist, incr_angle, start_point, angle_min):
		position=start_point+index

		angle=incr_angle*position+angle_min

		x=dist*np.cos(angle)

		y=dist*np.sin(angle)



		return x,y


	def scan_plotter(self, orig_ranges,filtered_ranges, incr_angle, start_point, angle_min):
		orig_x=[]
		orig_y=[]
		fltr_x=[]
		fltr_y=[]
		x_y_data=[]
		for i in range(len(orig_ranges)):
			x,y=self.dist_angle_to_xy_transform(i, orig_ranges[i], incr_angle, start_point, angle_min)
			orig_x.append(x)
			orig_y.append(y)
		for i in range(len(filtered_ranges)):
			x,y=self.dist_angle_to_xy_transform(i, filtered_ranges[i], incr_angle, start_point, angle_min)
			fltr_x.append(x)
			fltr_y.append(y)
		plt.clf()
		plt.plot(orig_x,orig_y, 'r.', fltr_x,fltr_y, 'b.')
		plt.axis([-30,30,-30,30])
		plt.draw()
		plt.show()
		
		return (fltr_x, fltr_y)

	def hsp_callback(self, msg):
		msg_len=len(msg.ranges) 

		start_time = rospy.Time.now();

		incr_angle=msg.angle_increment
		angle_min=msg.angle_min
		full_range=msg.angle_max-msg.angle_min #The full sweep angle of the laser scanner


		start_point=int((abs(msg.angle_min)+self.min_angle)/full_range*msg_len)
		end_point=int((full_range-self.max_angle)/full_range*msg_len) #

		ranges=msg.ranges
		'''
		my_ranges=[0]*len(ranges)
		for i in range(len(ranges)):
			if ranges[i]>msg.range_max:
				my_ranges[i]=msg.range_max
			elif ranges[i]<msg.range_min:
				my_ranges[i]=msg.range_min
			else:
				my_ranges[i]=ranges[i]
		'''
		filtered_ranges=ranges
		#filtered_ranges=self.filter(my_ranges)
		#print filtered_ranges
		#message=((self.min_angle,self.max_angle),filtered_ranges)
		#point_cloud=self.scan_plotter(ranges,filtered_ranges, incr_angle, start_point, angle_min)
		#self.point_cloud.publish(point_cloud)
		#print message
		#self.range_pub.publish(message)

		point_cloud=PointCloud()
		#setting up point cloud

		for i in range(len(filtered_ranges)):
			x,y=self.dist_angle_to_xy_transform(i, filtered_ranges[i], incr_angle, 0, angle_min)
			point = Point32();
			point.x = x;
			point.y=y;
			point.z=0.0
			point_cloud.points.append(point);


			
		point_cloud.header.frame_id = msg.header.frame_id
		point_cloud.header.stamp=rospy.Time.now()

		self.point_cloud.publish(point_cloud)
		timer = rospy.Time.now()-start_time


		rospy.loginfo("Timer: %f" % (timer.to_sec()))


if __name__ == "__main__":
	rospy.init_node("HokuyoScanProcessor")
	node=HokuyoScanProcessor()
	rospy.spin()

