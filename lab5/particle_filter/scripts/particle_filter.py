#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import random
import math
from sensor_msgs.msg import LaserScan

def get_ang_xy(x, y):
    #helper function to get angle from vx and vy, taken from online template
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

def add_noise(level, *coords):
    #helper function to add in random noise
    return [x + random.uniform(-level, level) for x in coords]


class Map2D(object):

    def __init__(self, iMap):
        self.occupancyArray= iMap.data
        self.width= iMap.info.width
        self.height= iMap.info.height
        self.resolution = iMap.info.resolution
        pass

    def map_data(self,x,y):
        return self.data[x*self.width+y]

    def map_valid(self,x,y):
        return (x<self.width and x>=0 and y<self.height and y>=0)

    def calc_range(robot_x,robot_y,robot_a,max_range):
        ##check if map frame is same as world frame, may not be
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

        while x != (x1 + xstep*1):
            x += xstep
            error += deltaerr
            if 2*error >= deltax:
                y += ystep
                error -= deltax

            # TODO: check
            # if steep:
            if not steep:
                if self.map_valid(y, x):
                    if self.map_data(x,y):
                        return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) /
                                self.scale)
            else:
                if self.map_valid(x, y):
                    if self.map_data[(x,y)]:
                        return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) /
                                self.scale)

        return max_range

    def world_to_map(self,x,y):
        ##check world to map transform
        mapx= math.floor(x/float(self.resolution))
        mapy= math.floor(y/float(self.resolution))



class Particle(object):
    ## Particle class, skeleton taken from template and modified to fit our purposes. Noisy left in as an option for now, may be removed
    def __init__(self, x, y, heading, w=0, 
                 noisy=False):
        if noisy:
            x, y, heading = add_noise(0.1, x, y, heading)

        self.x = x
        self.y = y
        self.h = heading
        self.w = w

    def __str__(self):
        return "(%f, %f, %f, w=%f)" % (self.x, self.y, self.h, self.w)

    @property
    def xyh(self):
        return self.x, self.y, self.h

    def motion_update(self, odom, step=0.02, noisy=False):
        #Takes in an odometry reading and a refresh rate(step) to return a new Particle that resulted from those actions in that time
        linV = odom.twist.twist.linear
        angV = odom.twist.twist.angular #Unused for right now
        speed = math.sqrt(linV.x**2 + linV.y**2)
        ang_xy= get_ang_xy(linV.x, linV.y) #There might be a better way to calculate angular velocity, currently using fn from online template

        h = self.h
        if noisy:
            speed, h = add_noise(0.02, speed, h)
        r = h + ang_xy

        dx = math.cos(r) * speed*step #Could use linV components but kept as speed for now to allow for noisy option
        dy = math.sin(r) * speed*step

        return Particle(self.x+dx, self.y+dy, r)






def MCL(iMap, particles, odom, sense): #This method should probably be moved to the ParticleFilter class eventually
    xBar=[] #Array for updated particles
    xReal=[] #Array for returned particles
    for p in particles:
        x=p.motion_update(lastOdom)
        x.w= sensor_update(iMap,sense, p)
        xBar.append(x)
    #Normalize weights
    total=0
    for p in xbar:
        total+=p.w
    for p in xbar:
        p.w=p.w/float(total)
    #Resample
    cdf=makecdf(xBar)
    for i in range(0,len(particles)):
        xReal.append(xBar[sample(cdf)]) #Could potentially cause issues of repeated references, may need to append a copy instead
    return xReal

def sensor_update(iMap, sense, p):
    ##sense is laserScan
    angle_min= sense.angle_min
    angle_increment= sense.angle_increment
    ranges = sense.ranges
    total=0
    for i in xrange(len(ranges)):
        p_d = iMap.calc_range(p.x, p.y, p.h+ angle_min + angle_increment*i, sense.max_range) ##probability of this measurement for this part of the laserscan
        total+=p_d
    return total/float(len(ranges)) ##updated particle weight


def makecdf(xBar):
    #makes a cdf from a list of particles with normalized weights
    total=0
    cdf=[]
    for p in xBar:
        total+=p.w
        cdf.append(total)
    return cdf

def sample(cdf):
    #returns index in array that was randomly chosen according to the distribution given by cdf
    r= random.random()
    for i in xrange(len(cdf)):
        if r > cdf[i]:
            return i-1
    return len(cdf)-1

class ParticleFilter:
    def __init__(self):

        #since laser data and odometry data will come in asynchronously, current idea is to store the last readings for use in a filter with a set refresh rate
        #May require some locking
        self.lastOdom=Odometry()
        self.lastLaser=LaserScan()

        self.topic_odom="/vesc/odom"
        self.topic_laser="scan"
        self.topic_map="map"
        #Pubs and Subs
        rospy.Subscriber(self.topic_odom, Odometry, self.odom_callback)
        cd_sub = rospy.Subscriber(self.topic_laser, LaserScan, self.laser_callback)

        #Filter init
        self.numParticles=10
        self.particles=[]
        self.minX=-10
        self.maxX=10
        self.minY=-10
        self.maxY=10
        for i in xrange(self.numParticles):
            x=random.randrange(self.minX,self.maxX)
            y=random.randrange(self.minY,self.maxY)
            theta = random.random()*2*math.pi
            self.particles.append(Particle(x,y,theta))


    def odom_callback(self, data):
        self.lastOdom=data

    def laser_callback(self,data):
        self.lastLaser=data

    def filter_step(self):
        pass

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("particle_filter")

    pF=ParticleFilter();

    #while not rospy.is_shutdown():
    #    pF.filter_step()
    #Commented out for now, should implement a refresh rate for the filter


    # enter the ROS main loop
    rospy.spin()