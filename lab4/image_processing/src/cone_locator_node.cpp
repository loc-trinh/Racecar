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
	image_transport::ImageTransport it;
	image_transport::Subscriber bw_image_sub; // bw_image
	ros::Publisher cone_location_pub;

	// callbacks
	void locationCallback(const sensor_msgs::ImageConstPtr& msg);

	// locals
	cv_bridge::CvImagePtr depth_image_ptr;
};

ConeLocatorNode::ConeLocatorNode() : it(nh) {
	bw_image_sub = it.subscribe("bw_image", 1, &ConeLocatorNode::locationCallback, this);
	cone_location_pub = nh.advertise<std_msgs::Float32>("cone_location", 1);
}

void ConeLocatorNode::locationCallback(const sensor_msgs::ImageConstPtr& msg){

	Mat bw_image;
	try{
		bw_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
	}
	catch (cv_bridge::Exception& e) {
  		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//find contours from binary image
    vector< vector<cv::Point> > contours;
    findContours(bw_image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); //find contours

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
        float angle = (center.x - W/2- W/5)/W *.959931;
    	std_msgs::Float32 angle_msg;
    	angle_msg.data = angle;

        ROS_INFO("FOUND OBJECT");
        cone_location_pub.publish(angle_msg);

    }else{
        ROS_INFO("CANNOT FIND OBJECT");
        std_msgs::Float32 angle_msg;
        angle_msg.data = 1000;
        cone_location_pub.publish(angle_msg);
    }
	return;
}


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "cone_locator_node");
	ConeLocatorNode node;
	ros::spin();
	return 0; 
}





