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

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
  int sign = 1;
  if (goal.pose.position.y > 0)
    sign = -1;


  /* Create new begin and end node */
  geometry_msgs::PoseStamped begin, end;
  begin.pose.position.x = .01;
  begin.pose.position.y = 0;
  end.pose.position.x = abs(goal.pose.position.y);
  end.pose.position.y = goal.pose.position.x;

  /* Log regression */
  double det = log(begin.pose.position.x) - log(end.pose.position.x);
  double a = (begin.pose.position.y - end.pose.position.y) / det;
  double b = (-log(end.pose.position.x)*begin.pose.position.y + log(begin.pose.position.x)*end.pose.position.y) / det;


  /* Generating path */
  plan.push_back(start);
  for (int i=1; i<50; i++){
    geometry_msgs::PoseStamped point;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.57079);
    point.pose.orientation.x = goal_quat.x();
    point.pose.orientation.y = goal_quat.y();
    point.pose.orientation.z = goal_quat.z();
    point.pose.orientation.w = goal_quat.w();

    double dx = (end.pose.position.x-begin.pose.position.x) * .02;
    double x = i * dx;
    point.pose.position.x = x * sign;
    point.pose.position.y = log(x) * a + b;
    plan.push_back(point);
  }
  plan.push_back(goal);
  return true;
  }
};