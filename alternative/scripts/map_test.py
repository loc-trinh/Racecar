#!/usr/bin/python
import rospy
import time
from Queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud
import numpy as np 
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
class TrajectoryFinder:
	def __init__(self):
		self.state_subs=rospy.Subscriber("state",Pose, self.statecallback)
		self.goal_subs=rospy.Subscriber("goal",Pose, self.goalcallback)
		self.map_subs=rospy.Subscriber("map",OccupancyGrid, self.mapcallback)
		self.cloud_pub=rospy.Publisher("cloud",PointCloud, queue_size=10)
		self.state=(17.496,-6.038)
		self.goal=(-1.112,-33.701)
		self.distance_weight=1
		self.goal_threshold=.5
		self.seq=0
	def goalcallback(self,data):
		self.goal=(data.position.Point.x,data.position.Point.y)
	def statecallback(self,data):
		self.goal=(data.position.Point.x,data.position.Point.y)
	def xy_to_grid(self,x,y):
		right=int(round((100+y)*20))
		up=int(round((100+x)*20))
		gridcell=right,up
		return gridcell
	def cell_cost(self,cell, data):
		cost=0
		if data[cell]<0:
			cost=100000.0
		else:
			cost=data[cell]
		return cost
	def heuristic(self, neighbor, goal):
		distance=self.distance(neighbor,goal)
		return distance*self.distance_weight
	def reconstruct(self,came_from,current):
		path=[current]
		while current in came_from.keys():
			current=came_from[current]
			path.append(current)
		return path
	def distance(self,neighbor,goal):
		dx=goal[0]-neighbor[0]
		dy=goal[1]-neighbor[1]
		distance=np.sqrt(dx**2+dy**2)
		return distance
	def neighbors(self, cell):
		dxs=[-1.6,0,1.6]
		x=cell[0]
		dys=[-1.6,0,1.6]
		y=cell[1]
		neighbors=[(x-1.6,y),(x-1.6,y+1.6),(x,y-1.6),(x-1.6,y-1.6),(x,y+1.6),(x+1.6,y-1.6),(x+1.6,y),(x+1.6,y+1.6)]
		
		return neighbors
	def mapcallback(self,data):
		mapdata=np.reshape(data.data,(4000,4000))
		m=self.xy_to_grid(self.goal[0],self.goal[1])
		chunk=[]
		# for i in range(m[0]-20,m[0]+20):
		# 	for j in range(m[1]-20,m[1]+20):
		# 		chunk.append(mapdata[i,j])
		# 		#print "chunk"
		# print chunk
		self.seq+=1
		frontier = PriorityQueue()
		frontier.put((0,self.state))
		closedSet={}
		openSet={}
		
		step_cost={}
		step_cost[self.state]=0
		came_from = {}					
		cost_so_far = {}
		came_from[self.state] = None
		cost_so_far[self.state] = self.heuristic(self.state, self.goal)
		openSet[self.state]=cost_so_far
		#print self.xy_to_grid(self.state[0],self.state[1])
		path=[]
		#x_list=[]
		#y_list=[]
		
		plt.axis([-40,40,-40,40])
		plt.ion()
		plt.show()
		terminate=False
		i=0
		while len(openSet)>0 and not terminate:
			current = min(openSet, key=openSet.get)
			if self.distance(current,self.goal)<=self.goal_threshold:
				path=self.reconstruct(came_from, current)
				print path
				terminate=True
			closedSet[current]=openSet.pop(current)
			for next in self.neighbors(current):
				if next in closedSet:
					continue
				x=next[0]
				y=next[1]
				# print x
				# print y
				# print came_from	
				#print self.cell_cost(self.xy_to_grid(x,y),mapdata)
				plt.plot(self.state[0],self.state[0],'or', self.goal[0],self.goal[1], 'og')
				plt.scatter(x,y)
				plt.draw()
				weight=10
				new_cost = step_cost[current] + self.distance(current,next)+self.cell_cost(self.xy_to_grid(x,y),mapdata)#+self.distance(current,next)*weight 	
				#print (new_cost,x,y)
				if next not in openSet:
					openSet[next]=new_cost+self.heuristic(next,self.goal)
				elif new_cost>=step_cost[next]:
					continue
				came_from[next]=current
				step_cost[next]=new_cost
				cost_so_far[next]=new_cost+self.heuristic(next,self.goal)

		time=rospy.Time.now()	
		cloud=PointCloud()
		cloud.header.seq=self.seq
		cloud.header.frame_id="1"
		cloud.header.stamp=time
		for i in range(len(path)-1):
			point=Point()
			point.x=path[i][0]
			point.y=path[i][1]
			point.z=0.0
			cloud.points.append(point);

		self.cloud_pub.publish(cloud)

    	
if __name__ == "__main__":
	rospy.init_node("TrajectoryFinder")
	node=TrajectoryFinder()
	rospy.spin()