


#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cmath>

class StereoFilter:
{
public:
	StereoFilter();
private:
	ros::NodeHandle nh;
	ros::Subscriber image_sub;
	pub::Publisher Image_pub;

	void imageCallback(sensor_msgs::Image::ImageConstPtr& msg);


};
int main(int argc, char *argc[])
{
	
	ros::inti(argc,argv,"image_filter_node");
	StereoFilter node;
	ros::spin();
	return 0;
}

StereoFilter::StereoFilter():
	
{
	//subcribe to the number stream topic
	image_sub=nh.subcribe("image", 1, imageCallback, this);
	
	ROS_INFO("Created timer with period of %f seconds", moving_average_period_);


}

void StereoFilter::imageCallback(const sensor_msgs::Image::ImageConstPtr& msg){


}