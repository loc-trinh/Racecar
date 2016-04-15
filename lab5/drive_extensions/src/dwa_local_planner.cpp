#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>

// 

using namespace std, ros;


class TrajectoryPlannerNode:public base_local_planner::TrajectoryPlanner {

public:
  TrajctoryPlannerNode();

private:
  ros::NodeHandle nh;

~<name>/global_plan (nav_msgs/Path)
The portion of the global plan that the local planner is currently attempting to follow. Used primarily for visualization purposes.
~<name>/local_plan (nav_msgs/Path)
The local plan or trajectory that scored the highest on the last cycle. Used primarily for visualization purposes.
~<name>/cost_cloud (sensor_msgs/PointCloud2)
The cost grid used for planning. Used for visualization purposes. See the publish_cost_grid_pc parameter for enabling/disabling this visualization. New in navigation 1.4.0

  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS costmap("my_costmap", tf);

  // Pubs and Subs
  ros::Subscriber sub = nh.subscribe("odom", 1000, chatterCallback);
  ros::Publisher global_plan_pub;
  ros::Publisher local_plan_pub;
  ros::Publisher cost_cloud_pub;

  dwa_local_planner::DWAPlannerROS dp;
  dp.initialize("my_dwa_planner", &tf, &costmap);


  // Callbacks
  void localPlanCallback(//need something here);

  // Locals
  // None needed?

};



void TrajectoryPlannerNode::localPlanCallback(const sensor_msgs::ImageConstPtr& msg){

ndContours(bw_image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); //find contours

    //find largest contour area
    double maxArea = -1;
    int max_index = -1;
    for(int i = 0; i < contours.size(); i++) {
        if (contourArea(Mat(contours[i])) > maxArea){
            maxArea = contourArea(Mat(contours[i]));
            max_index = i;
        }
    }

    cv::Point center;
    Rect r;
    if (maxArea > 1000) {
        r = boundingRect(contours[max_index]);
        center.x = r.x + (r.width/2);
        center.y= r.y + (r.height/2);
        float W = bw_image.cols;
        float angle = (center.x - W/2)/W *.959931;
      std_msgs::Float32 angle_msg;
      angle_msg.data = -angle;

        ROS_INFO("FOUND OBJECT");
        cone_location_pub.publish(angle_msg);

        std_msgs::Float32 x_msg;
        x_msg.data = center.x;
        x_pub.publish(x_msg);

    }else{
        ROS_INFO("CANNOT FIND OBJECT");
        std_msgs::Float32 angle_msg;
        angle_msg.data = 1000;
        cone_location_pub.publish(angle_msg);
    }
  return;
}


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "base_local_planner_node");
  TrajectoryPlannerNode node;
  ros::spin();
  return 0; 
}
