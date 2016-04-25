#include <stdio.h>
#include <ros/ros.h>

using namespace std;

// Alternative local planner
// Input: 
//		trajectory (vector of points) from global planner 
//		local cost map (lidar data)
// Output:
// 		trajectory to goal points, modified for obstacles
//		(stretch) add in cost computation for colored circle detection

// Suggested logic - detect obstacle and then plot a course that just goes right next to it
// Or a discrete version of TEB? aka draw a line, find obstacles, and keep moving around, then sample it back?
// Or could create two nodes that subscribe and funnel into a final node that passes everything into my node lawl

class AltLocalPlannerNode { 
public:
	AltLocalPlannerNode(); // constructor

private:
	ros::NodeHandle nh;

	// pubs-subs

	// Update this to receive a trajectory (vector of points) from Loc
	ros::Subscriber globalPathSub;

	// Update this to receive a cost map from global planner
	ros::Subscriber localCostMapSub;

	// Update this to link into the direct ackermann commands
	ros::Publisher localDriveComdsPub;

	// Double check how to chain together multiple subscriptions, store cost map as local var?
	// or lock it up?

	// callbacks
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void someOtherCallback(const );
};

AltLocalPlannerNode::AltLocalPlannerNode() : it(nh) {
	globalPathSub = it.subscribe("/??", 1, &AltLocalPlannerNode::someCallback, this);
	localCostMapSub = it.subscribe("??", 1, &AltLocalPlannerNode::someOtherCallback, this);
	localDriveComdsPub = it.advertise("localDriveCommands", 1);
}



int main(int argc, char *argv[]) {
	ros::init(argc, argv, "AltLocalPlannerNode");
	AltLocalPlannerNode node;
	ros::spin();
	return 0; 
}
