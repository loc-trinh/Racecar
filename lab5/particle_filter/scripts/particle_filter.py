#!/usr/bin/python
import rospy
import random
import math
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg


def add_noise_helper(level, *coords):
    ## Helper function to add in random noise
    return [x + random.uniform(-level, level) for x in coords]

class Particle(object):
    ## Particle class. Noisy left in as an option for now, may be removed
    def __init__(self, x, y, heading, w=0, noisy=False):
        if noisy:
            x, y, heading = add_noise_helper(0.1, x, y, heading)

        self.x = x
        self.y = y
        self.h = heading
        self.w = w

    def motion_update(self, odom, step=0.1, noisy=False):
        # Takes in an odometry reading and a refresh rate(step) to return a new Particle that resulted from those actions in that time
        linV = odom.twist.twist.linear
        angV = odom.twist.twist.angular #unused
        speed = math.sqrt(linV.x**2 + linV.y**2)
        ang_xy = self.get_ang_xy(linV.x, linV.y)

        h = self.h
        if noisy:
            speed, h = add_noise_helper(0.02, speed, h)
        r = h + ang_xy

        # Could use linV components but kept as speed for now to allow for noisy option
        dx = math.cos(r) * speed*step
        dy = math.sin(r) * speed*step

        return Particle(self.x+dx, self.y+dy, r)

    def get_ang_xy(self, x, y):
        # Helper function to get angle from vx and vy, taken from online template
        ang = 0
        if x != 0:
            ang = math.atan(math.fabs(y / x))
            if x < 0:
                ang = math.pi - ang
        else:
            ang = math.pi / 2
        if y < 0:
            ang = -1 * ang
        return ang

    def __str__(self):
        return "(%f, %f, %f, weight=%f)" % (self.x, self.y, self.h, self.w)

    @property
    def xyh(self):
        return (self.x, self.y, self.h)

class Map(object):
    def __init__(self, iMap):
        self.occupancyArray= iMap.data
        self.width= iMap.info.width
        self.height= iMap.info.height
        self.resolution = iMap.info.resolution
        self.origin_x = iMap.info.origin.position.x
        self.origin_y = iMap.info.origin.position.y

    def map_data(self,x,y):
        return self.occupancyArray[int(x)*self.width+int(y)]

    def map_valid(self,x,y):
        return (x<self.width and x>=0 and y<self.height and y>=0)

    def calc_range(self, robot_x,robot_y,robot_a,max_range):
        x0, y0 = self.world_to_map(robot_x, robot_y)
        x1, y1 = self.world_to_map(robot_x + max_range*math.cos(robot_a),
                                   robot_y + max_range*math.sin(robot_a))

        if abs(y1-y0) > abs(x1-x0):
            steep = True
        else:
            steep = False

        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1

        deltax = abs(x1-x0)
        deltay = abs(y1-y0)
        error = 0
        deltaerr = deltay

        x = x0
        y = y0

        if x0 < x1:
            xstep = 1
        else:
            xstep = -1
        if y0 < y1:
            ystep = 1
        else:
            ystep = -1

        if steep:
            if not self.map_valid(y, x) or self.map_data(y,x) == -1 or self.map_data(y,x) > 50:
                return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * self.resolution)
        else:
            if not self.map_valid(x, y) or self.map_data(x,y) == -1 or self.map_data(x,y) > 50:
                return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * self.resolution)

        while x != (x1 + xstep*1):
            x += xstep
            error += deltaerr
            if 2*error >= deltax:
                y += ystep
                error -= deltax

            if steep:
                if not self.map_valid(y, x) or self.map_data(y,x) == -1 or self.map_data(y,x) > 50:
                    return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * self.resolution)
            else:
                if not self.map_valid(x, y) or self.map_data(x,y) == -1 or self.map_data(x,y) > 50:
                    return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * self.resolution)

        return max_range

    def world_to_map(self,x,y):
        ## World to map transformation
        mapx = math.floor((x - self.origin_x) / float(self.resolution) + 0.5) + self.width / 2
        mapy = math.floor((y - self.origin_y) / float(self.resolution) + 0.5) + self.height / 2
        return (mapx, mapy)

def gaussian(mean, variance, value):
    # Helper function to calculate probability from guassian distribution
    return 1/math.sqrt(2*math.pi*variance) * math.exp(-(mean-value)**2/2/variance)

def sensor_update(iMap, sense, p):
    total=0
    for i in xrange(len(sense.ranges)):
        expectated = iMap.calc_range(p.x, p.y, p.h + sense.angle_min + sense.angle_increment*i, sense.range_max)
        observed = sense.ranges[i]

        mean, variance = expectated, 1.0
        total += gaussian(mean, variance, observed)
    return total

def makecdf(X_bar):
    # Helper function: makes a cdf from a list of particles with normalized weights
    total = 0
    cdf = []
    for p in X_bar:
        total += p.w
        cdf.append(total)
    return cdf

def sample(cdf):
    # Helper function: Sample index randomly chosen according to the distribution given by cdf
    r = random.random()
    for i in xrange(len(cdf)):
        if r < cdf[i]:
            return i-1
    return len(cdf)-1

class ParticleFilter:
    def __init__(self):
        # Since laser data and odometry data will come in asynchronously, current idea is to store the last readings for use in a filter with a set refresh rate
        # May require some locking
        self.lastOdom = Odometry()
        self.lastLaser = LaserScan()

        #Pubs and Subs
        self.topic_odom = "/vesc/odom"
        self.topic_laser = "scan"
        rospy.Subscriber(self.topic_odom, Odometry, self.odom_callback)
        rospy.Subscriber(self.topic_laser, LaserScan, self.laser_callback)
        self.particles_pub = rospy.Publisher("/particles", PointCloud, queue_size=10)

        #Services
        self.map = None
        rospy.wait_for_service('/static_map')
        self.map = Map(rospy.ServiceProxy('/static_map', GetMap)().map)
        self.lastScanTime=rospy.Time.now()
      
        #Filter init
        self.numParticles = 100
        self.particles = []
        self.minX = -15 + self.map.origin_x
        self.maxX = 15 + self.map.origin_x
        self.minY = -15 + self.map.origin_y
        self.maxY = 15 + self.map.origin_y
        for i in xrange(self.numParticles):
            x = random.randrange(self.minX,self.maxX)
            y = random.randrange(self.minY,self.maxY)
            theta = random.random()*2*math.pi
            self.particles.append(Particle(x,y,theta))

    def odom_callback(self, data):
        self.lastOdom = data

    def laser_callback(self,data):
        self.lastLaser = data

        if self.map != None:
			print "======= GOT MAP ========"
			print data.scan_time
			self.filter_step()
			for i in self.particles:
				print str(i),
			print
			#declaring pointcloud
			pointcloud = PointCloud()
			#filling pointcloud header
			header = std_msgs.msg.Header()
			header.stamp = rospy.Time.now()
			header.frame_id = 'odom'
			pointcloud.header = header
			#filling some points
			for i in self.particles:
				pointcloud.points.append(Point32(i.x,i.y,0))
			#publish
			self.particles_pub.publish(pointcloud)

    def filter_step(self):
    	if self.map == None:
    		return

        # MCL implementation
        X_bar = []
        X = []

        for p in self.particles:
            x = p.motion_update(self.lastOdom, step = (rospy.Time.now()-self.lastScanTime).to_sec())
            x.w = sensor_update(self.map, self.lastLaser, p)
            X_bar.append(x)

        #Normalize weights
        total = sum([p.w for p in X_bar])
        for p in X_bar:
            p.w = p.w/float(total)
            print p.w

        #Resample
        cdf = makecdf(X_bar)
        for i in range(self.numParticles):
            p = X_bar[sample(cdf)]
            X.append(Particle(p.x,p.y,p.h,p.w)) 

        self.particles = X
        self.lastScanTme=rospy.Time.now()


if __name__ == "__main__":
    
    rospy.init_node("particle_filter")

    pF = ParticleFilter()

    #while not rospy.is_shutdown():
    #    pF.filter_step()
    #Commented out for now, should implement a refresh rate for the filter

    # enter the ROS main loop
    rospy.spin()