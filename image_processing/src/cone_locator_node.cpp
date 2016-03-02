#include <stdio.h>
#include <ros/ros.h> // main ROS include
#include <geometry_msgs/Point.h>
#include <math.h>

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
Input: b/w RGB image, depth_image image
converting to Mat (OpenCV)

Output: Point(x,y) of the center of mass, converted to real world coords

This node flood fills (find the boundaries of) the cone in the b/w image 
Returns the center of mass of the desired object

Probs not Loc(k):
http://answers.ros.org/question/28373/race-conditions-in-callbacks/
*/

class ConeLocatorNode { 
public:
	ConeLocatorNode(); // constructor

private:
	ros::NodeHandle nh;

	// pubs-subs
	image_transport::ImageTransport it;
	image_transport::Subscriber bw_image_sub; // bw_image
	image_transport::Subscriber depth_image_sub; // depth_image
	ros::Publisher cone_location_pub;

	// callbacks
	void locationCallback(const sensor_msgs::ImageConstPtr& msg);
	void depthCallback(const sensor_msgs::ImageConstPtr& msg);

	// locals
	cv_bridge::CvImagePtr depth_image_ptr;
};

ConeLocatorNode::ConeLocatorNode() : it(nh) {
	bw_image_sub = it.subscribe("bw_image", 1, &ConeLocatorNode::locationCallback, this);
	depth_image_sub = it.subscribe("/camera/zed/depth/image_rect_color", 1, &ConeLocatorNode::depthCallback, this);
	cone_location_pub = nh.advertise<std_msgs::Float32>("cone_location", 1);
}

void ConeLocatorNode::depthCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		depth_image_ptr = cv_bridge::toCvCopy(msg);
	}
	catch (cv_bridge::Exception& e) {
  		ROS_ERROR("cv_bridge exception DEPTH: %s", e.what());
		return;}
	return;
}

void ConeLocatorNode::locationCallback(const sensor_msgs::ImageConstPtr& msg){
	geometry_msgs::Point coords;

	// if (!depth_image_ptr) return;
	// not sure how to access the depth_image here using the ptr?
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

    //drawContours(bw_image, contours, max_index, Scalar(255), CV_FILLED);

    cv::Point center;
    Rect r;
    if (maxArea > 1000) {
        r = boundingRect(contours[max_index]);

        // Find the length of the x, approximate as a portion of the fov arc
    	int arc = (r.x + (r.width/2)) / r.width;
    	angle = arc * 110;

        ROS_INFO("FOUND OBJECT");
        cone_location_pub.publish(angle);

    }else{
        ROS_INFO("CANNOT FIND OBJECT");
        coords.x = -1;
        coords.y = -1;
        coords.z = -1;
        cone_location_pub.publish(coords);
    }
	return;
}


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "cone_locator_node");
	ConeLocatorNode node;
	ros::spin();
	return 0; 
}





