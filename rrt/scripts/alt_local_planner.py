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
# 	OccupancyGrid (base_link)
# 	Global Trajectory (10 PoseStamped from a Path)
# Output:
# 	updated 10 pt trajectory to be sent to a trajectory follower


# Helper Functions:
#	snag tf and stuff? from Reo's 

# https://github.mit.edu/rss2016-team9/racecar/blob/mubarik_alt_global/navstack/scripts/backup_recovery.py

# eventually going to be merged with the freespace detector, remember to add pruning for max turning angle and free space evals
# also check to see if impossible to reach point, take over for the backup recovery blah pull that code in?

class LocalPlannerNode:
    def __init__(self):
        self.newPath = []
        self.globalPath = []
        self.grid = None

        self.base_frame = rospy.get_param('~base_frame', self.base_frame)

     	#Pubs & Subs
     	rospy.Subscriber("/costmap_base/costmap/costmap", OccupancyGrid, self.costmap_callback)
     	rospy.Subscriber("/costmap_base/costmap/costmap_updates", OccupancyGridUpdate, self.costmap_update_callback)
        
        # Need to add subscription to global Path trajectory
        rospy.Subscriber("somethinghere", Path, self.trajectory_callback)

        # Need to update publishing to trajectory follower
        self.drive_pub = rospy.Publisher("/trajFollower", Path, queue_size=1)

        # Tf Stuff
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

    def check_trajectory(self):
    	# first point in trajectory is current loc, add to newPath
    	self.newPath[0] = self.globalPath[0]

    	# for each point in the global trajectory after the first once
    		# check if the current dest point is valid
    		# check if the points in between are valid (reachability)
    		# if either check violated, one-step branch
    			# sample random range of points
    			# order by proximity to current point
    			# pick closest point that is free
    			# add new dest point to newPath
    		# else, append current dest point from globalPath to newPath

    	for pose_ind in xrange(1,len(globalPath)):
			branch = False

			# lookup dest and find its value
			dest = globalPath[pose_ind].pose.position
			dest_grid = self.positionToIndex(dest.x, dest.y)
			if self.grid.data[dest_grid] > 0:
				branch = True

			# sample points between current newPath and next dest

			# find slope

			if self.grid.data[grid_ind] > 0:
				branch = True
				break

			# branch (helper function?)

			float ratio = (grid_x2 - grid_x1) / (grid_y2 - grid_y1)
			float width = grid_y2 - grid_y1;
			for(int i = 0; i < width; i++) {
			    float new_y = grid_y1 + i;
			    float new_x = grid_x1 + (ratio * i);
			    if (map[new_x * map.info.width + new_y] > threshold);
			    	branch = true;


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

	if __name__ == "__main__":
		rospy.init_node("local_planner_node")
		local_planner = LocalPlannerNode()
	    rate = rospy.Rate(60)
	    while not rospy.is_shutdown():
	        # update and add-in recovery behavior
	        # update with the name of the function
	        local_planner.do_something();
	        rate.sleep()