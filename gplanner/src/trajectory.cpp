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

	/* Create new begin and end node tranforms */
	geometry_msgs::TransformStamped transform;
	transform = tfBuffer.lookupTransform("base_link", start.header.frame_id, ros::Time(0),ros::Duration(1));
	tf2::doTransform(start, begin, transform);
	transform = tfBuffer.lookupTransform("base_link", goal.header.frame_id, ros::Time(0),ros::Duration(1));
	tf2::doTransform(goal, end, transform);
	try{
      transform = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(1));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Curve_Planner: %s",ex.what());
      ros::Duration(1.0).sleep();
      
    }

    /* Creating plan */
	double dx = end.pose.position.x - begin.pose.position.x;
	double dy = end.pose.position.y - begin.pose.position.y;

	geometry_msgs::PoseStamped prev_point = begin;

	tf2::doTransform(begin, begin, transform);
	plan.push_back(begin);
	//if (abs(dy) < 1){
		
		for(int i = 0; i < step; i++){
			
			geometry_msgs::PoseStamped point = end;
			

		    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(atan2(dx,dy));
		    point.pose.orientation.x = goal_quat.x();
		    point.pose.orientation.y = goal_quat.y();
		    point.pose.orientation.z = goal_quat.z();
		    point.pose.orientation.w = goal_quat.w();

		    point.pose.position.x = prev_point.pose.position.x + 1/step * dx;
		    point.pose.position.y = prev_point.pose.position.y + 1/step * dy;

		    prev_point = point;

		    tf2::doTransform(point, point, transform);
			plan.push_back(point);
		}
	//}
	tf2::doTransform(end, end, transform);
	plan.push_back(end);
	
	base_local_planner::publishPlan(plan, global_plan_pub_); 
	return true;
  }
};