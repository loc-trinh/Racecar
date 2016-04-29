#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <tuple>

// includes for ROS message types
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

#include <vector>
// #include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

// Alternative local planner (Baby RRT)
// Subscribes: 
//		trajectory (array of 10 PoseStamped) from Loc
//		local cost map (lidar data) from costmap node (Loc)
// Publishes:
// 		trajectory vector of 10 PostStamped msgs
// Algorithm:
//		(stretch) add in cost computation for colored circle detection

// Need to make sure node spins at certain frequency, instead of working off of callbacks

class AltLocalPlannerNode { 
public:
	AltLocalPlannerNode(); // constructor

private:
	ros::NodeHandle nh;

	// pubs-subs
	ros::Subscriber globalPathSub;
	ros::Subscriber localCostMapSub;
	ros::Publisher localPathPub;

	// callbacks
	// update when figure out what type the trajectory is
	void trajectoryCallback(const nav_msgs::Path& msg);
	void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

	// locals
	nav_msgs::OccupancyGrid::ConstPtr localCostMapPtr;
	nav_msgs::Path trajPtr;

AltLocalPlannerNode::AltLocalPlannerNode() : it(nh) {
	globalPathSub = it.subscribe("/somewhere", 1, &AltLocalPlannerNode::trajectoryCallback, this);
	localCostMapSub = it.subscribe("overtherainbow/grid", 1, &AltLocalPlannerNode::costMapCallback, this);
	localDriveComdsPub = it.advertise("localDriveCommands", 1);
}

void AltLocalPlannerNode::trajectorycallback(const geometry_msgs_vector::ConstPtr &msg) {
	// not sure how to store the info from a pointer so that it can be used by the class?
	trajPtr = msg.data;

	// change possible array type to vector?
	// tf transform the posestamped into base_link?
}

void AltLocalPlannerNode::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	// again, not sure how to store things in C++?
	localCostMapPtr = msg.data;

	// do the tf transforms here on the occupancygrid?
}

/* Params: not sure how to do params in C++? http://wiki.ros.org/roscpp/Overview/Parameter%20Server
	stepSize: defaults to Euclidean distance between first two pts in trajectory
	branchFactor: defaults to 5 new waypoints to consider

	Algorithm:
	Similar to RRT, takes in trajectory and checks if each point is valid and the
	straight line path there is valid. If not, build a tree that allows the car to go
	forward/around obstacle, then take each of these new trajectories, and try
	the next point in the trajectory, iterate until find valid path to goal point.

	how to pass in cost map/locations?
*/
void AltLocalPlannerNode::pathPlanning(OccupancyGrid map, Path traj){
	// v is a vector of PostStamped
	int delta = v.at(1).data.pose.point.x - v.at(0).data.pose.point.x
	tuple<int,int,int> point (10,10,10);

	// check on this param
	int branchFactor = 5;
	std::vector<tuple<int,int,int>> myvector (branchFactor);

	// threshold should be a param? currently set in config file?
	int threshold = 0.35;

	for(std::vector<T>::iterator it = v.begin(); it != v.end(); ++it) {
    	// std::cout << *it;

    	// check current point's value in occupancy grid
    	// if invalid, branch, else, move to next point
    	grid_x = (unsigned int)((*it.data.pose.point.x - map.info.origin.position.x) / map.info.resolution);
    	grid_y = (unsigned int)((*it.data.pose.point.y - map.info.origin.position.y) / map.info.resolution);  
		int occ_val = map[grid_x * width + grid_y];

		if (occ_val > threshold):

     // how to access costmap data?
     // if point in invalid part of cost map,
     // sample points in range, delta ahead of car
     // prune these points,
     // append to an array
     // 




	}

}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "AltLocalPlannerNode");
	AltLocalPlannerNode node;
	ros::Rate r(60); // 60 hz
	// call the planning func here?
	ros::spin();
	return 0; 
}
