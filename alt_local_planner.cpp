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
	std_msgs::Header pathHeader;	
	vector<geometry_msgs::PoseStamped> path;
	nav_msgs::MapMetadata metaMapInfo;
	std_msgs::Header mapHeader;
	nav_msgs::OccupancyGrid map;

AltLocalPlannerNode::AltLocalPlannerNode() : it(nh) {
	globalPathSub = it.subscribe("/somewhere", 1, &AltLocalPlannerNode::trajectoryCallback, this);
	localCostMapSub = it.subscribe("overtherainbow/grid", 1, &AltLocalPlannerNode::costMapCallback, this);
	localDriveComdsPub = it.advertise("localDriveCommands", 1);
}

void AltLocalPlannerNode::trajectorycallback(const nav_msgs::Path::ConstPtr &msg) {
	std_msgs::Header pathHeader = msg.header;	
	vector<geometry_msgs::PoseStamped> path = msg.poses;

	// tf transform the posestamped into base_link?
}

void AltLocalPlannerNode::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
	nav_msgs::MapMetadata metaMapInfo = msg.info;
	std_msgs::Header mapHeader = msg.header;
	nav_msgs::OccupancyGrid map = msg.data;
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
void AltLocalPlannerNode::pathPlanning(){
	// v is a vector of PostStamped
	int delta = path.at(1).data.pose.point.x - path.at(0).data.pose.point.x
	// tuple<int,int,int> point (10,10,10);
	// std::vector<tuple<int,int,int>> myvector (branchFactor);

	// these should be params? threshold currently set in config file?
	int threshold = 0.35;
	int branchFactor = 5;

	// need to initialize new path?

	// iterate over all points in the path
	for(std::vector<geometry_msgs::PoseStamped>::size_type i = 0; i != path.size(); i++) {
	    /* std::cout << someVector[i]; ... */

		// add logic here - if current point has already been overruled, or we should consider the one in the new path instead, add decision here
		

    	// check each traj point's value in occupancy grid
    	grid_x1 = (unsigned int)((*path[i].data.pose.point.x - map.info.origin.position.x) / map.info.resolution);
    	grid_y1 = (unsigned int)((*path[i].data.pose.point.y - map.info.origin.position.y) / map.info.resolution);  
		int occVal1 = map[grid_x1 * map.info.width + grid_y1];

		// check dest point's val in occupancy grid, if not the last point
		if (i != path.size()) {
			grid_x2 = (unsigned int)((*path[i+1].data.pose.point.x - map.info.origin.position.x) / map.info.resolution);
	    	grid_y2 = (unsigned int)((*path[i+1].data.pose.point.y - map.info.origin.position.y) / map.info.resolution);  
			int occVal2 = map[grid_x2 * map.info.width + grid_y2];	
		} 
		
		// check the path between them in the occupancy grid
		new vector<>


		// if origin, dest, or path between invalid, branch
		if (occVal1 > threshold) or (occVal2 > threshold) or (branch):
			// randomly sample from [0,1]*delta
			// sort by closest
			// add these to the dest point, check if valid
			// if valid, 

     // how to access costmap data?
     // if point in invalid part of cost map,
     // sample points in range, delta ahead of car
     // prune these points,
     // append to an array
     

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
