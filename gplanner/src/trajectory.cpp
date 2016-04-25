#include <pluginlib/class_list_macros.h>
#include "trajectory.h"
#include <math.h>
//#include <tf/transform_listener.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

GlobalPlanner::GlobalPlanner (){}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){initialize(name, costmap_ros);}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  ros::NodeHandle nh("~/" + name);
  global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
	double step = 10;

	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
	geometry_msgs::PoseStamped begin, end;

	/* Create new begin and end node */
	geometry_msgs::TransformStamped transform;
	transform = tfBuffer.lookupTransform("base_link", start.header.frame_id, ros::Time(0),ros::Duration(1));
	tf2::doTransform(start, begin, transform);
	tf2::doTransform(goal, end, transform);
	try{
      transform = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(1));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Curve_Planner: %s",ex.what());
      ros::Duration(1.0).sleep();
      
    }

    /*Transform incoming message from header.frame_id to base_link
	listener.waitForTransform("base_link", start.header.frame_id, ros::Time(0), ros::Duration(10.0) );
	tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
	listener.transformPose("base_link", start, begin);

	listener.waitForTransform("base_link", goal.header.frame_id, ros::Time(0), ros::Duration(10.0) );
	listener.transformPose("base_link", goal, end);
	*/

	double deltax = abs(begin.pose.position.x-end.pose.position.x);
	tf2::doTransform(begin, begin, transform);
	plan.push_back(begin);

 	//Straight Line
	if (deltax < 1){
		for(int i = 0; i < step; i++){
			double dy = (end.pose.position.y-start.pose.position.y)/float(step);
			double m = float(end.pose.position.x-start.pose.position.x)/(end.pose.position.y-start.pose.position.y);
			
			geometry_msgs::PoseStamped point = end;
		    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
		    point.pose.orientation.x = goal_quat.x();
		    point.pose.orientation.y = goal_quat.y();
		    point.pose.orientation.z = goal_quat.z();
		    point.pose.orientation.w = goal_quat.w();

		    point.pose.position.y = begin.pose.position.y + i * dy;
		    point.pose.position.x = m * point.pose.position.x;

		    //listener.transformPose("odom", point, point);
		    tf2::doTransform(point, point, transform);
			plan.push_back(point);
		}
	}
	tf2::doTransform(end, end, transform);
	plan.push_back(end);
 //  // Curved Line
	// else{
	// 	int sign = 1;
	// 	if (end.pose.position.x < 0){
	// 		end.pose.position.x = -end.pose.position.x;
	// 		sign = -1;
	// 	}

	// 	/* Log regression */
	// 	double det = log(begin.pose.position.x) - log(end.pose.position.x);
	// 	double a = (begin.pose.position.y - end.pose.position.y) / det;
	// 	double b = (-log(end.pose.position.x)*begin.pose.position.y + log(begin.pose.position.x)*end.pose.position.y) / det;

	// 	/* Generating path */
	// 	tf2::doTransform(begin, begin, transform);
	// 	plan.push_back(begin);
	// 	for (int i=1; i < step; i++){
	// 		geometry_msgs::PoseStamped point = goal;
	// 		tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
	// 		point.pose.orientation.x = goal_quat.x();
	// 		point.pose.orientation.y = goal_quat.y();
	// 		point.pose.orientation.z = goal_quat.z();
	// 		point.pose.orientation.w = goal_quat.w();

	// 		double dx = (end.pose.position.x-begin.pose.position.x) * 1 / step;
	// 		double x = i*dx;
	// 		point.pose.position.y = -x * sign;
	// 		point.pose.position.x = log(x) * a + b;

	// 	    tf2::doTransform(point, point, transform);
	// 		plan.push_back(point);
	// 	}
	// 	tf2::doTransform(end, end, transform);
	// 	double temp = end.pose.position.x;
	// 	end.pose.position.x = end.pose.position.y;
	// 	end.pose.position.y = -temp;
	// 	plan.push_back(end);
	// }
	
	base_local_planner::publishPlan(plan, global_plan_pub_); 
	return true;
  }
};