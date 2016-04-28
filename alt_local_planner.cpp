#include <stdio.h>
#include <ros/ros.h>

// includes for ROS message types
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs_vector.h"

using namespace std;

// Alternative local planner (Baby RRT)
// Subscribes: 
//		trajectory (vector of 10 PoseStamped) from Loc
//		local cost map (lidar data) from costmap node (Loc)
// Publishes:
// 		trajectory vector of 10 PostStamped msgs
// Algorithm:
//		(stretch) add in cost computation for colored circle detection

// Need to make sure node spins at certain frequency, instead of working off of callbacks

// void ConeLocatorNode::locationCallback(const sensor_msgs::ImageConstPtr& msg){

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
	void trajectoryCallback(const sensor_msgs::ImageConstPtr& msg);
	void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

	// locals
	nav_msgs::OccupancyGrid::ConstPtr localCostMapPtr;
	vector<geometry_msgs::PoseStamped::ConstPtr> trajPtr;

AltLocalPlannerNode::AltLocalPlannerNode() : it(nh) {
	globalPathSub = it.subscribe("/somewhere", 1, &AltLocalPlannerNode::trajectoryCallback, this);
	localCostMapSub = it.subscribe("overtherainbow/grid", 1, &AltLocalPlannerNode::costMapCallback, this);
	localDriveComdsPub = it.advertise("localDriveCommands", 1);
}

void AltLocalPlannerNode::trajectorycallback(const package_name::geometry_msgs_vector::ConstPtr &msg) {
}

void AltLocalPlannerNode::costMapCallback(const sensor_msgs::ImageConstPtr& msg){

}

/* Params:
	stepSize: defaults to Euclidean distance between first two pts in trajectory
	branchFactor: defaults to 5 new waypoints to consider

	Algorithm:
	Similar to RRT, takes in trajectory and checks if each point is valid and the
	straight line path there is valid. If not, build a tree that allows the car to go
	forward/around obstacle, then take each of these new trajectories, and try
	the next point in the trajectory, iterate until find valid path to goal point.
*/
void AltLocalPlannerNode::pathPlanning("blah"){

}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "AltLocalPlannerNode");
	AltLocalPlannerNode node;
	ros::Rate r(60); // 60 hz
	// call the planning func here?
	ros::spin();
	return 0; 
}
