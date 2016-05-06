#!/usr/bin/python
import rospy
import numpy as np 
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
class WallDetector:
	def __init__(self):
		
		self.subs=rospy.Subscriber("scan", LaserScan, self.wallcallback)
		self.pubs=rospy.Publisher("walldata",Point32,queue_size=4)

		plt.ion()
		self.dist=3.0
		self.left_end=5*np.pi/9
		self.left_start=np.pi/12
		self.right_end=-np.pi/12
		self.right_start=-5*np.pi/9
	def diff_dist(self,ar,al,br,bl,cl,cr,d, x):
		d1=(abs(ar*x-np.sqrt(d**2-x**2)+cr)/np.sqrt(ar**2+1))
		d2=(abs(al*x-np.sqrt(d**2-x**2)+cl)/np.sqrt(al**2+1))
		print "d1:",d1,"d2: ",d2
		return d1-d2

	def dist_angle_to_xy_transform(self, index, dist, incr_angle, start_point, angle_min):
		position=start_point+index

		angle=incr_angle*position+angle_min

		x=dist*np.sin(angle)

		y=dist*np.cos(angle)
		return x,y 
	def wallcallback(self,data):

		#Find the locations of the two walls
		left_end=int((self.left_end-data.angle_min)/(data.angle_max-data.angle_min)*len(data.ranges))
		left_start=int((self.left_start-data.angle_min)/(data.angle_max-data.angle_min)*len(data.ranges))
		right_end=int((self.right_end+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		right_start=int((self.right_start+data.angle_max)/(data.angle_max-data.angle_min)*len(data.ranges))
		right_wall=data.ranges[right_start:right_end]
		left_wall=data.ranges[left_start:left_end]

		#filter out infinities 

		# for i in range(len(data.ranges[right_start:right_end])-2):
		# 	#print min(data.ranges[right_start:right_end][i:i+2])
		# 	right_wall.append(min(data.ranges[right_start:right_end][i:i+2]))
		# 	left_wall.append(min(data.ranges[left_start:left_end][i:i+2]))


		print "angle_min,angle_max: ",data.angle_min,data.angle_max
		print "range: ", len(data.ranges)
		#find the x-y transformation of the walldata
		print "leftstart:", left_start, "leftend: ",left_end, "right_start: ",right_start,"right_end: ", right_end
		right_wallx=[]
		right_wally=[]
		for i in range(len(right_wall)):
			if right_wall[i]<8.0:
				rx,ry=self.dist_angle_to_xy_transform(i, right_wall[i], data.angle_increment, right_start, data.angle_min)
				right_wallx.append(rx)
				right_wally.append(ry)
		left_wallx=[]
		left_wally=[]
		for i in range(len(left_wall)):
			if left_wall[i]<8.0:
				lx,ly=self.dist_angle_to_xy_transform(i, left_wall[i], data.angle_increment, left_start, data.angle_min)
				left_wallx.append(lx)
				left_wally.append(ly)
		print "left_wallx:" ,len(left_wallx),"left_wally: ",len(left_wally), "right_wallx: ",len(right_wallx), "right_wally: ",len(right_wally)
		left_wall_plot,residuals_l, rank_l, singular_values_l, rcond_l=np.polyfit(left_wallx,left_wally,1,full=True)
		right_wall_plot,residuals_r, rank_r, singular_values_r, rcond_r=np.polyfit(right_wallx,right_wally,1,full=True)
		residuals_l = residuals_l/len(left_wallx)
		residuals_r = residuals_r/len(right_wallx)
		#slopes and interepts of the two walls 
		rSlope=right_wall_plot[0]
		rIntercept=right_wall_plot[1]
		lSlope=left_wall_plot[0]
		lIntercept=left_wall_plot[1]
		#find the point 
		ar,al,cr,cl=rSlope,lSlope,rIntercept,lIntercept
		br,bl=(ar)**2+1,(al)**2+1
		


		d=self.dist

		x=0.0
		current=abs(self.diff_dist(ar,al,br,bl,cl,cr,d, x))
		step=.02
		changing=True
		while changing:
			#print "Here"
			down=abs(self.diff_dist(ar,al,br,bl,cl,cr,d, x-step*(abs(x-step)<=d)))
			up=abs(self.diff_dist(ar,al,br,bl,cl,cr,d, x+step*(x+step<d)))
			if min(down,up,current)==current:
				changing=False
			current=min(down,up,current)
			if current==down:
				x=x-step
			elif current==up:
				x=x+step 
		print "current: ", current, "x: ",x 

		ycpl=x
		xcpl=np.sqrt(d**2-x**2)
		point=Point32()
		point.x=abs(xcpl)
		point.y=-ycpl
		point.z=0.0
		self.pubs.publish(point)









		# plt.show()
		# left_line=[]
		# right_line=[]
		# for i in range(len(left_wallx)):
		# 	left_line.append(left_wallx[i]*left_wall_plot[0]+left_wall_plot[1])
		# for i in range(len(right_wallx)):
		# 	right_line.append(right_wallx[i]*right_wall_plot[0]+right_wall_plot[1])
		# a=(left_wall_plot[0]+right_wall_plot[0])/2.0
		# b=(left_wall_plot[1]+right_wall_plot[1])/2.0
		# y=3.0
		# x=(y-b)/(a+.0001)
		# # x=3.0*np.sqrt(1/(1+(sl)**2))
		# # y=-x*sl 
		# # x=-x
		# print "goalx: ",x, "goaly: ",y
		# xo=0.0
		# yo=0.0
		# plt.plot(left_wallx,left_wally,'.')
		# plt.plot(right_wallx,right_wally,'.')
		# #plt.plot(xy,yx, '.')
		# plt.plot(ycpl,xcpl,"*")
		# #plot(my_y,d)
		# plt.plot(xo,yo, '^')
		# plt.plot(left_wallx, left_line)
		# plt.plot(right_wallx,right_line)
		# plt.axis([-10,10,-10,10])
		
		# plt.draw()
		# # # plt.show()

if __name__ == "__main__":
	rospy.init_node("WallDetector")
	node=WallDetector()
	rospy.spin()