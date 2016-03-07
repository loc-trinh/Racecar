#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// Interfacing to OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// For Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class ImageFilterNode { 
public:
	ImageFilterNode(); // constructor

private:
	ros::NodeHandle nh;

	// pubs-subs
	image_transport::ImageTransport it;
	image_transport::Subscriber camera_sub;
	image_transport::Publisher image_pub;

	// callbacks
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

ImageFilterNode::ImageFilterNode() : it(nh) {
	camera_sub = it.subscribe("/camera/zed/rgb/image_rect_color", 1, &ImageFilterNode::imageCallback, this);
	image_pub = it.advertise("bw_image", 1);
}

void ImageFilterNode::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr image_pointer;
	try{
		image_pointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
  		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
    cvtColor(image_pointer->image, image_pointer->image, COLOR_BGR2HSV);
    inRange(image_pointer->image, cv::Scalar(0, 80, 80), cv::Scalar(22, 255, 255), image_pointer->image);
    image_pub.publish(image_pointer->toImageMsg());   
}


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "image_filter_node");
	ImageFilterNode node;
	ros::spin();
	return 0; 
}





