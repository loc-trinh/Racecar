#include <pluginlib/class_list_macros.h>
#include "trajectory.h"
#include <math.h>

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
	double step = 20;

	/* Create new begin and end node */
	geometry_msgs::PoseStamped begin = start;
	geometry_msgs::PoseStamped end = goal;
	begin.pose.position.x = .01;
	begin.pose.position.y = 0;
	end.pose.position.x = -goal.pose.position.y;
	end.pose.position.y = goal.pose.position.x;

	double deltax = abs(start.pose.position.x-end.pose.position.x);

  //Straight Line
	if (deltax < 1){
		plan.push_back(start);
		for(int i = 0; i < step; i++){
			double dx = (end.pose.position.x-start.pose.position.x)/float(step);
			double m = float(end.pose.position.y-start.pose.position.y)/(end.pose.position.x-start.pose.position.x);
			geometry_msgs::PoseStamped point = goal;
		    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
		    point.pose.orientation.x = goal_quat.x();
		    point.pose.orientation.y = goal_quat.y();
		    point.pose.orientation.z = goal_quat.z();
		    point.pose.orientation.w = goal_quat.w();

		    point.pose.position.x = i * dx;
		    point.pose.position.y = m * point.pose.position.x;
			plan.push_back(point);
		}
		plan.push_back(end);
	}
  // Curved Line
	else{
		int sign = 1;
		if (end.pose.position.x < 0){
			end.pose.position.x = -end.pose.position.x;
			sign = -1;
		}

		/* Log regression */
		double det = log(begin.pose.position.x) - log(end.pose.position.x);
		double a = (begin.pose.position.y - end.pose.position.y) / det;
		double b = (-log(end.pose.position.x)*begin.pose.position.y + log(begin.pose.position.x)*end.pose.position.y) / det;

		/* Generating path */
    plan.push_back(start);
		for (int i=1; i < step; i++){
			geometry_msgs::PoseStamped point = goal;
			tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
			point.pose.orientation.x = goal_quat.x();
			point.pose.orientation.y = goal_quat.y();
			point.pose.orientation.z = goal_quat.z();
			point.pose.orientation.w = goal_quat.w();

			double dx = (end.pose.position.x-begin.pose.position.x) * 1 / step;
			double x = i*dx;
			point.pose.position.y = x * sign;
			point.pose.position.x = log(x) * a + b;
			plan.push_back(point);
		}
		end.pose.position.x = -end.pose.position.x;
		plan.push_back(end);
	}
	for(int i = 0; i < plan.size(); i++){
		double temp = plan[i].pose.position.x;
		plan[i].pose.position.x = plan[i].pose.position.y;
		plan[i].pose.position.y = -temp;
	}
	base_local_planner::publishPlan(plan, global_plan_pub_); 
	return true;
  }
};