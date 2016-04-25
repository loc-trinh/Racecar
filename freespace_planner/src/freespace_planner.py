#!/usr/bin/python

###############
##
## Freespace Planner
## 
## A really basic goal-generator based on 
## available free-space up ahead.
##
###############

# Python Includes
import numpy as np
import math

# ROS Includes
import rospy
import tf

# ROS messages
#from geometry_msgs.msg import PoseArray, Pose, PointStamped
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal


class FreespacePlanner:
    def __init__(self):

        #Class members
        self.grid = None

        #Default settings
        self.kfactor = 0.6;
        self.topic_occgrid = "/move_base/global_costmap/costmap";
        self.topic_occgridupdates = "/move_base/global_costmap/costmap_updates";
        self.topic_output = "/move_base_simple/goal";
        self.base_frame = "base_link"
        self.map_frame = "odom";

        #Load Settings
        self.kfactor = rospy.get_param('~kfactor', self.kfactor)
        self.topic_occgrid = rospy.get_param('~topic_occgrid', self.topic_occgrid)
        self.topic_occgridupdates = rospy.get_param('~topic_occgridupdates', self.topic_occgridupdates)
        self.topic_output = rospy.get_param('~topic_output', self.topic_output)
        self.base_frame = rospy.get_param('~base_frame', self.base_frame)
        self.map_frame = rospy.get_param('~map_frame', self.map_frame)

        #Pubs and Subs
        self.goal_pub = rospy.Publisher(self.topic_output, PoseStamped, queue_size=10)
        rospy.Subscriber(self.topic_occgrid, OccupancyGrid, self.costmap_callback)
        rospy.Subscriber(self.topic_occgridupdates, OccupancyGridUpdate, self.costmap_update_callback)

        #Setup TF
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

        rospy.loginfo("FreeSpace Planner node loaded")

    def get_cell_range(self,pt1, pt2, meta):
        #1 - Convert points to map coords
        pt1[0] -= meta.origin.position.x;
        pt1[1] -= meta.origin.position.y;
        pt2[0] -= meta.origin.position.x;
        pt2[1] -= meta.origin.position.y;

        #2 - convert coords to rows and cols
        row_start  = int(pt1[0]*(1/meta.resolution));
        row_end = int(pt2[0]*(1/meta.resolution));
        col_start  = int(pt1[1]*(1/meta.resolution));
        col_end = int(pt2[1]*(1/meta.resolution));


        #3 - Create cell array
        #cells = [0] * ((row_end - row_start)*(col_end * col_start));
        cells = []
        for col in range(col_start, col_end):
            for row in range(row_start, row_end):
                cells.append(self.getIndex(col,row));
        return cells

    def count(self,grid,cells):
        unknown = 0
        empty = 0
        full = 0;
        for cell in cells:
            if grid[cell] == -1:
                unknown+=1;
            elif grid[cell] == 0: 
                empty +=1;
            else:
                full += 1
        return (unknown, empty, full);

    def perform_update(self):
        print "==============="
        if self.grid == None:
            print "Waiting for Initial OCC Grid..."
            return
        # 1. Get cells for left, right, and center segments:
        left_cells = self.get_cell_range([1,0],[4,3],self.grid.info)
        right_cells = self.get_cell_range([-4,0],[-1,3],self.grid.info)
        closecenter_cells = self.get_cell_range([-1,0],[1,1.0],self.grid.info)
        farcenter_cells = self.get_cell_range([-1,1.0],[1,4],self.grid.info)

        # 2. count it up
        unknown = [0.0] * 4
        empty = [0.0] * 4
        full = [0.0] * 4
        (unknown[0], empty[0], full[0]) = self.count(self.grid.data, left_cells)
        (unknown[1], empty[1], full[1]) = self.count(self.grid.data, closecenter_cells)
        (unknown[2], empty[2], full[2]) = self.count(self.grid.data, farcenter_cells)
        (unknown[3], empty[3], full[3]) = self.count(self.grid.data, right_cells)

        left_free = float(empty[0]) / (empty[0]+unknown[0]+full[0]);
        right_free = float(empty[3]) / (empty[3]+unknown[3]+full[3]);


        if empty[1]+full[1] >0:
            center_close = 1 - (float(full[1]) / (empty[1]+full[1]))
        else: 
            center_far=0;
        if empty[2]+full[2] >0:
            center_far = 1 - (float(full[2]) / (empty[2]+full[2]))
        else: 
            center_far=0;
            
        center_ranking = 0.7*center_close + 0.3*center_far;

        print "Left: = %d, %d, %d" % (unknown[0], empty[0], full[0])
        print "Center_Close: = %d, %d, %d" % (unknown[1], empty[1], full[1])
        print "Center_Far: = %d, %d, %d" % (unknown[2], empty[2], full[2])
        print "Right: = %d, %d, %d" % (unknown[3], empty[3], full[3])
        print "-------"
        print "L Ranking = %f" % left_free
        print "R Ranking = %f" % right_free
        print "C Ranking = %f" % center_far
        print "Center Navigable = %f" % center_close

        if center_close < 0.5:
            x = -0.5;
        else:
            x = 3.5*center_far

        if right_free > 0.3:
            y = 2
        else:
            y=(left_free-right_free)*4;

        if(x < 0.1):
            y = y*4;

        #msg = MoveBaseGoal()

        goal = PoseStamped()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0#math.atan2(y,x)
        goal.header.frame_id = 'base_link'

        goal.header.stamp = rospy.Time.now()
        #msg.target_pose = goal
        self.goal_pub.publish(goal)

        return

    def getIndex(self,x,y):
        sx = self.grid.info.width;
        return y * sx + x;

    def costmap_update_callback(self,data):
        if self.grid == None:
            print "Waiting for Initial OCC Grid..."
            return
        i =0;
        for y in range(data.y, data.y+data.height):
            for x in range(data.x, data.x+data.width):
                self.grid.data[self.getIndex(x,y)] = data.data[i];
                i+=1

    def costmap_callback(self, data):
        self.grid = data;
        self.grid.data = list(self.grid.data)

    
if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("freespace_planner")

    Planner = FreespacePlanner();
    #sleep(1);

    #rospy.spin();
    
    # enter the ROS main loop
    
    rate = rospy.Rate(0.333)
    while not rospy.is_shutdown():
        Planner.perform_update();
        rate.sleep()