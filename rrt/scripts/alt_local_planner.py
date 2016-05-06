#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf
import math

# Input: 
#   OccupancyGrid (base_link)
#   Global Trajectory (10 PoseStamped from a Path)
# Output:
#   updated 10 pt trajectory to be sent to a trajectory follower

# Helper Functions:
#   check_trajectory - takes in globalPath and outputs new trajectory
#   branch - one-step RRT, takes in two endpoints of a traj and returns PointStamped
#   positionToIndex - takes two x,y coords in base_link and calls getIndex
#   getIndex - takes two x,y coords and turns translates to OccupancyGrid index
#   check_inbetween - takes a prev and dest Point and samples the costmap inbetween them

# Callbacks:
#   trajectory_callback - stores globalPath as a list of PoseStamped
#   costmap_update_callback - stores OccupancyGrid in self.grid via looping 
#   costmap_callback - stores Occupancy in self.grid as a list

# https://github.mit.edu/rss2016-team9/racecar/blob/mubarik_alt_global/navstack/scripts/backup_recovery.py

# Progress:
#   Needs to be tested
#   Needs to prune for max-turning/destination impossible to reach
#   Needs to be merged with the freespace detector
#   Needs to be merged with backup

class LocalPlannerNode:
    def __init__(self):
        self.newPath = []
        self.globalPath = []
        self.grid = None
        self.base_frame = "base_link"

        self.base_frame = rospy.get_param('~base_frame', self.base_frame)

        #Pubs & Subs
        rospy.Subscriber("/costmap_base/costmap/costmap", OccupancyGrid, self.costmap_callback)
        rospy.Subscriber("/costmap_base/costmap/costmap_updates", OccupancyGridUpdate, self.costmap_update_callback)
        
        # Need to add subscription to global Path trajectory
        rospy.Subscriber("global_plan", Path, self.trajectory_callback)

        # Need to update publishing to trajectory follower
        self.drive_pub = rospy.Publisher("local_plan", Path, queue_size=1)

        # Tf Stuff
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

    def check_trajectory(self):
	if not self.globalPath:
		print "Waiting on trajectory"
		return 0

        # first point in trajectory is current loc, add to newPath
        self.newPath.append(self.globalPath[0])

        # for each point in the global trajectory after the first once
            # check if the current dest point is valid
            # check if the points in between are valid (reachability)
            # if either check violated, one-step branch
                # compute y (ROS's +x) distance
                # sample random range of points at this dist
                # order by proximity to current point
                    # prune for max steering angle (0.25 radians)
                # pick closest point that is free
                # add new dest point to newPath
                # TODO - infeasibility of final goal
            # else, append current dest point from globalPath to newPath

        for pose_ind in xrange(1,len(globalPath)):
            prev = newPath[pose_ind-1].pose.position

            # lookup dest and find its value
            branch_dest = False
            dest = globalPath[pose_ind].pose.position
            dest_grid = self.positionToIndex(dest.x, dest.y)
            if self.grid.data[dest_grid] > 0:
                branch_dest = True

            # sample points between current newPath pose and next dest
            branch_between = check_inbetween(prev, dest)

            # if need to branch for either dest or inbetween, run helper, 
            # else append current path to newPath
            if branch_dest or branch_between:
                new_dest = self.branch(newPath[pose_ind-1].pose.position, dest)
                newPath.append(new_dest)
            else:
                # need to update point header or...?
                newPath.append(globalPath[pose_ind])

    def check_inbetween(self, prev, dest):
        branch = False
        inbetween = []
        # find slope and sample some points to next dest
        slope = float(dest.y - prev.y) / (dest.x - prev.x)
        partition = 10
        delta = float(sqrt(sum(dest.x-prev.x)**2,(dest.y-prev.y)**2) / partition)
        # generate inbetween points and check their costs
        for i in xrange(partition):
            sample_x = prev.x + delta
            sample_y = prev.y + ratio * delta
            inbetween.append((sample_x, sample_y))
            prev.x = sample_x
            prev.y = sample_y
        for point in inbetween:
            grid_ind = positionToIndex(point[0], point[1])
            if self.grid.data[grid_ind] > 0:
                branch = True
                break
        return branch

    # one-step RRT branch, returns PoseStamped
    def branch(self, prev, dest):
        # change limits when adding bounding box

        # sampling width is controlled by max steering angle (radians)
        max_steering_angle = 0.25
        depth = dest.y - prev.y
        width = depth * math.tan(max_steering_angle)

        # create list of new possible destinations
        rands = []
        while len(rands) < 10:
            sample_dest = (dest.x+random.uniform(0, width), dest.y)
            cost_of_dest = self.grid.data[positionToIndex(sample_dest[0], sample_dest[1])]
            branch = check_inbetween(prev, sample_dest)
            if (cost_of_dest > 0) and (not branch):
                rands.append(new_dest)
        
        # choose the point closest to initial trajectory
        new_dest = min(rands, key=lambda x: abs(x[0]-dest.x))

        # create new PoseStamped
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = new_dest[0]
        pose.pose.position.y = new_dest[1]
        pose.header.frame_id = self.base_frame
        pose.header.stamp = rospy.Time.now()

        return pose

    # Converts base_link position to index in occupancygrid
    def positionToIndex(self,x,y):
        x -= self.grid.info.origin.position.x
        y -= self.grid.info.origin.position.y
        x = int(x*(1/self.grid.info.resolution));
        y  = int(y*(1/self.grid.info.resolution));
        sx = self.grid.info.width;
        return int(round(y * sx + x));

    def trajectory_callback(self, data):
        self.globalPath = list(data.poses)

    def costmap_update_callback(self,data):
        if self.grid == None:
            print "Waiting for Initial OCC Grid..."
            return
        i = 0;
        for y in range(data.y, data.y+data.height):
            for x in range(data.x, data.x+data.width):
                self.grid.data[self.getIndex(x,y)] = data.data[i];
                i+=1

    def costmap_callback(self, data):
        self.grid = data;
        self.grid.data = list(self.grid.data)

    def getIndex(self,x,y):
        sx = self.grid.info.width;
        return int(round(y * sx + x));

if __name__ == "__main__":
    rospy.init_node("local_planner_node")
    local_planner = LocalPlannerNode()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        # update and add-in recovery behavior
        local_planner.check_trajectory();
        rate.sleep()
