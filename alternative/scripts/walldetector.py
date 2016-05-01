#!/usr/bin/python
import rospy
import numpy as np 
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
class WallDetector:
	def __init__(self):
		
		self.subs=rospy.Subscriber("scan", LaserScan, self.wallcallback)
		self.right_slope=rospy.Publisher("walldata/rslope",Float32, queue_size=1)
		self.right_intercept=rospy.Publisher("walldata/rintercept",Float32, queue_size=1)
		self.l_slope=rospy.Publisher("walldata/lslope",Float32, queue_size=1)
		self.l_intercept=rospy.Publisher("walldata/lintercept",Float32, queue_size=1)

		plt.ion()
		self.dist=3.0
		self.left_end=2*np.pi/3
		self.left_start=np.pi/12
		self.right_end=-np.pi/12
		self.right_start=-2*np.pi/3
	def diff_dist(self,ar,al,br,bl,cl,cr,d, x):
		d1=(abs(ar*x-np.sqrt(d**2-x**2)+cr)/np.sqrt(ar**2+1))
		d2=(abs(al*x-np.sqrt(d**2-x**2)+cl)/np.sqrt(ar**2+1))
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
		right_wall=[]#data.ranges[right_start:right_end]
		left_wall=[]#data.ranges[left_start:left_end]

		#filter out infinities 

		for i in range(len(data.ranges[right_start:right_end])-2):
			#print min(data.ranges[right_start:right_end][i:i+2])
			right_wall.append(min(data.ranges[right_start:right_end][i:i+2]))
			left_wall.append(min(data.ranges[left_start:left_end][i:i+2]))



		#find the x-y transformation of the walldata
		print "leftstart:", left_start, "leftend: ",left_end, "right_start: ",right_start,"right_end: ", right_end
		right_wallx=[]
		right_wally=[]
		for i in range(len(right_wall)):
			if right_wall[i]<6.0:
				rx,ry=self.dist_angle_to_xy_transform(i, right_wall[i], data.angle_increment, right_start, data.angle_min)
				right_wallx.append(rx)
				right_wally.append(ry)
		left_wallx=[]
		left_wally=[]
		for i in range(len(left_wall)):
			if left_wall[i]<6.0:
				lx,ly=self.dist_angle_to_xy_transform(i, left_wall[i], data.angle_increment, left_start, data.angle_min)
				left_wallx.append(lx)
				left_wally.append(ly)
		print "left_wallx:" ,len(left_wallx),"left_wally: ",len(left_wally), "right_wallx: ",len(right_wallx), "right_wally: ",len(right_wally)
		left_wall_plot=np.polyfit(left_wallx,left_wally,1).tolist()
		right_wall_plot=np.polyfit(right_wallx,right_wally,1).tolist()
		#publish [[right_wall_slope,right_wall_intercept],[left_wall_slope,left_wall_intercept]]
		rightSlope=right_wall_plot[0]
		rightIntercept=right_wall_plot[1]
		leftSlope=left_wall_plot[0]
		leftIntercept=left_wall_plot[1]
		self.right_slope.publish(rightSlope)
		self.right_intercept.publish(rightIntercept)
		self.l_slope.publish(leftSlope)
		self.l_intercept.publish(leftIntercept)

		ar,al,cr,cl=rightSlope,leftSlope,rightIntercept,leftIntercept
		br,bl=(ar)**2+1,(al)**2+1
		#g=br*cl**2-bl*cr**2 


		d=self.dist

		# a=(ar**2-al**2)
		# b=2*((al**2*(d+cr)+cr)-(ar**2*(d+cl)+cl))
		# c=(al**2+1)(d+cr)**2-(al**2+1)*(d+cl)**2
		# my_y=0
		# yn=(-b-np.sqrt(b**2-4*a*c))/(2.0*a)
		# yp=(-b+np.sqrt(b**2-4*a*c))/(2.0*a)
		# dn=abs(self.diff_dist(ar,al,br,bl,cl,cr,d, yn))
		# dp=abs(self.diff_dist(ar,al,br,bl,cl,cr,d, yp))

		# if dn<dp:
		# 	my_y=yn
		# else:
		# 	my_y=yp

		# a1,a2,b1,b2,c1,c2=ar,al,br,bl,cr,cl
		x=0.0
		current=abs(self.diff_dist(ar,al,br,bl,cl,cr,d, x))
		step=.02
		changing=True
		##for i in range(int(d*1/(2.0*step))):
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
		# p1=2*b2*a1 
		# p2=2*b1*a2 
		# q1=2*b2*c1 
		# q2=2*b1*c2 

		# q=q1-q2
		# p=p1-p2

		# m=b2*(a1**2-1)-b1*(a2**2-1)
		# n=2*a1*b2*c1-2*a2*b1*c2 
		# g=b1*c2-b2*c1-d**2*(b2-b1)

		# a=(m**2-p**2)
		# b=2*m*n-2*p*q
		# c=d**2*p**2-q**2+n**2-2*g*m 
		# d=2*d**2*p*q-2*g*n
		# e=g**2+d**2*q**2








		# p1,p2=((b+c))**2/(4*a**2),np.sqrt((27*a*d**2+27*e*(b+c)**2)**2-4*(12*a*e-3*d*(b+c))**3)

		# p3,p4=27*a*d**2+27*e*(b+c)**2,(2**(1/3.0)*(4*a*e-b*d-c*d))

		# p5,p6,p7,p8=-((b+c)**3)/a**3-8*d/a, -(b+c)/(4*a),(p2+p3)**(1/3.0),1.0/(3*a*2**(1/3.0))

		# x1=1/2.0*np.sqrt((p1+1/(3*a*2**(1/3.0))*(p7))+p4/(a*(p7)))
		# x2=1/2.0*np.sqrt(p1*2+p5/(4*np.sqrt(p1+1/(3*a*2**(1/3.0))*p7+p3/p7))-1/(1/(3*a*2**(1/3.0)))*p7-p4/(a*p7))-(b+c)/(4*a)
		# xa=x1-x2
		# print "Mycomplicatedx:",xa
		# x1=-1/2.0*np.sqrt(p1+p8*p7+p4/(a*p7))-1/2.0*np.sqrt((p1*2)+p5/(4*np.sqrt(p1+p8*p7+p4/(a*p7)))-p8*p7-p4/(a*p7))+p6

		# x2=-1/2.0*np.sqrt(p1+p8*p7+p4/(a*p7))+1/2.0*np.sqrt((p1*2)+p5/(4*np.sqrt(p1+p8*p7+p4/(a*p7)))-p8*p7-p4/(a*p7))+p6

		# x3=1/2.0*np.sqrt(p1+p8*p7+p4/(a*p7))-1/2.0*np.sqrt((p1*2)+p5/(4*np.sqrt(p1+p8*p7+p4/(a*p7)))-p8*p7-p4/(a*p7))+p6

		# x4=1/2.0*np.sqrt(p1+p8*p7+p4/(a*p7))+1/2.0*np.sqrt((p1*2)+p5/(4*np.sqrt(p1+p8*p7+p4/(a*p7)))-p8*p7-p4/(a*p7))+p6

		# print "x1,x2,x3,x4: ",x1,x2,x3,x4


		mystart=510
		xy=[]
		yx=[]
		for i in range(30):
			lx,ly=self.dist_angle_to_xy_transform(i, data.ranges[mystart+i], data.angle_increment, mystart, data.angle_min)
			xy.append(lx)
			yx.append(ly)









		# ar,al,cr,cl=rightSlope,leftSlope,rightIntercept,leftIntercept
		# br,bl=(ar)**2+1,(al)**2+1
		# g=br*cl**2-bl*cr**2 


		# # c1=ar*bl*(2*cr*((self.dist)**2-2)/np.sqrt((self.dist)**2-1.0)+ar*self.dist)
		# # c2=al*br*(2*cl*((self.dist)**2-2)/np.sqrt((self.dist)**2-1.0)+al*self.dist)
		# # cg=c1-c2
		# cg=ar*bl*(2*cr*np.sqrt((self.dist)**2-1.0)+ar*self.dist)-al*br*(2*cl*np.sqrt((self.dist)**2-1.0)+al*self.dist)
		# # b1=2*bl*(ar*(self.dist-2.0)/np.sqrt((self.dist)**2-1.0)+cr*(np.sqrt((self.dist)**2-1.0)+1)/np.sqrt((self.dist)**2-1.0))
		# # b2=2*br*(al*((self.dist)**2-2.0)/np.sqrt((self.dist)**2-1.0)+cl*(np.sqrt((self.dist)**2-1.0)+1)/np.sqrt((self.dist)**2-1.0))
		# # b=b1-b2

		# c=cg-g
		# b=2*bl*(ar*np.sqrt((self.dist)**2-1.0)+cr)-2*br*(al*np.sqrt((self.dist)**2-1.0)+cl)
		# # a1=bl*((np.sqrt(self.dist-1.0)+1)/np.sqrt((self.dist)**2-1.0)-ar)
		# # a2=br*((np.sqrt(self.dist-1.0)+1)/np.sqrt((self.dist)**2-1.0)-al)
		# # a=a1-a2
		# a=bl*(1-ar)-br*(1-al)

		# xn=(-b-np.sqrt(b**2-4*c*a))/(2.0*a)
		# xp=(-b+np.sqrt(b**2-4*c*a))/(2.0*a)
		# print "xnegative,xpositive:",xn,xp
		# plt.scatter(left_wallx,right_wally,'.')
		# plt.scatter(right_wallx,right_wally,'.')
		# plt.plot(left_wallx,leftSlope*left_wallx+leftIntercept,'-')
		# #plt.plot(right_wallx,rightSlope*right_wallx+rightIntercept,'-')
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
		# plt.plot(xy,yx, '.')
		# plt.plot(ycpl,xcpl,"*")
		# #plot(my_y,d)
		# plt.plot(x,y,'.',xo,yo, '^')
		# plt.plot(left_wallx, left_line)
		# plt.plot(right_wallx,right_line)
		# plt.axis([-10,10,-10,10])
		
		# plt.draw()
		# # # plt.show()

if __name__ == "__main__":
	rospy.init_node("WallDetector")
	node=WallDetector()
	rospy.spin()