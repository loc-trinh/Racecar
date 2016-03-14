import rospy
from nav_msgs import Odometry
import random
import math

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
            # needs more noise to disperse better
            # h += random.uniform(-3, 3)
        r = h + ang_xy

        dx = math.cos(r) * speed*step #Could use linV components but kept as speed for now to allow for noisy option
        dy = math.sin(r) * speed*step

        return Particle(self.x+dx, self.y+dy, r)






def MCL(iMap, particles, odom, sense): #This method should probably be moved to the ParticleFilter class eventually
    xBar=[] #Array for updated particles
    xReal=[] #Array for returned particles
    for p in particles:
        x=p.motion_update(lastOdom)
        weight= sensor_update(iMap,sense, particles[m])
        xBar.append((x,weight))
    #Normalize weights
    total=0
    for pair in xbar:
        total+=pair[1]
    for pair in xbar:
        pair[1]=pair[1]/float(total)
    #Resample
    for i in range(0,len(particles)):
        xReal.append(sample(xBar))
    return xReal


def sensor_update(iMap,sense, particle):
    pass

def sample(xBar):
    pass


class ParticleFilter:
    def __init__(self):
        self.lastOdom=Odometry()
        self.topic_odom="/vesc/odom"
        #Pubs and Subs
        rospy.Subscriber(self.topic_odom, Odometry, self.odom_callback)

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
            theta=random.randrange(0, 2*math.pi)
            self.particles.append(Particle(x,y,theta))


    def odom_callback(self, data):
        self.lastOdom=data

    def filter_step(self):
        pass

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("path_planner")

    pF=ParticleFilter();

    # while not rospy.is_shutdown():
    #     pF.filter_step()
    #Commented out for now, should implement a refresh rate for the filter


    # enter the ROS main loop
    rospy.spin()