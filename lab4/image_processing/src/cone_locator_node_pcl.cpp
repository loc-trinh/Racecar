#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// PCL Includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

/*
Input: b/w RGB image as Mat (Open CV)

Output: An angle from the car to the target cone

This node flood fills (find the boundaries of) the cone in the b/w image.
If there are multiple cones, we treat the blob as a super-cone, and do the same flood fill.
*/

class ConeLocatorNode { 
public:
	ConeLocatorNode(); // constructor

private:
	ros::NodeHandle nh;

	// pubs-subs
	ros::Subscriber pcl_cloud; // pointcloud sub, ZED camera outputs a pcl2
	ros::Publisher cone_pcl;

	// callbacks
	void pclCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&);

};

ConeLocatorNode::ConeLocatorNode() : it(nh) {
	// conv to pc from pcl2 should be done on the fly by the subscriber
	pcl_cloud_sub = it.subscribe("zed/camera/point_cloud/cloud", 1, &ConeLocatorNode::pclCallback, this);
	cone_pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_processing", 1);
}

void ConeLocatorNode::pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_input){


pcl::PointCloud<pcl::PointXYZRGB>
	sensor_msgs::PointCloud2 pcl_processing;
	pcl_processing = *pcl_input;
    cone_location_pub.publish(pcl_processing);

	return;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "cone_locator_node");
	ConeLocatorNode node;
	ros::spin();
	return 0; 
}





