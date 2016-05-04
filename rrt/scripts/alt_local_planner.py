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
#   snag tf and stuff? from Reo's 

# https://github.mit.edu/rss2016-team9/racecar/blob/mubarik_alt_global/navstack/scripts/backup_recovery.py

# eventually going to be merged with the freespace detector, remember to add pruning for max turning angle and free space evals
# also check to see if impossible to reach point, take over for the backup recovery blah pull that code in?

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
        # first point in trajectory is current loc, add to newPath
        self.newPath[0] = self.globalPath[0]

        # for each point in the global trajectory after the first once
            # check if the current dest point is valid
            # check if the points in between are valid (reachability)
            # if either check violated, one-step branch
                # compute y (ROS's +x) distance
                # sample random range of points at this dist
                # order by proximity to current point
                # pick closest point that is free
                # add new dest point to newPath
                # TODO - prune for max steering angle/infeasibility of final goal
            # else, append current dest point from globalPath to newPath

        for pose_ind in xrange(1,len(globalPath)):
            branch = False

            prev = newPath[pose_ind-1].pose.position

            # lookup dest and find its value
            dest = globalPath[pose_ind].pose.position
            dest_grid = self.positionToIndex(dest.x, dest.y)
            if self.grid.data[dest_grid] > 0:
                branch = True

            # sample points between current newPath and next dest
            # find slope and sample some points to next dest
            inbetween = []
            slope = float(dest.y - prev.y) / (dest.x - prev.x)
            partition = 10
            delta = float(sqrt(sum(dest.x-prev.x)**2,(dest.y-prev.y)**2) / partition)
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

            # if need to branch, run helper, else append current path to newPath
            if branch:
                new_dest = self.branch(newPath[pose_ind-1].pose.position, dest)
                newPath.append(new_dest)
            else:
                # need to update point header or...?
                newPath.append(globalPath[pose_ind])

    # one-step RRT branch, returns PoseStamped
    def branch(self, prev, dest):
        # change when adding bounding box
        # dist = dest.y-prev.y
        # dude what's a reasonable width for this?
        width = self.grid.info.width / 6

        rands = []
        for i in xrange(10):
            sample_dest = (dest.x+random.uniform(0, width), dest.y)
            cost_of_dest = self.grid.data[positionToIndex(sample_dest[0], sample_dest[1])]
            if cost_of_dest > 0:
                rands.append(new_dest)
        
        new_dest = min(rands, key=lambda x: x[0])

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
        # update with the name of the function
        local_planner.do_something();
        rate.sleep()