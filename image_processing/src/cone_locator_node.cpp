#include <stdio.h>
#include <ros/ros.h> // main ROS include
#include <geometry_msgs/Point.h>

// Interfacing to OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// For Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//For flood fill


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
void myfloodFill(Mat& image, int r, int c, int& area, int& high_x, int& low_x, int& high_y, int& low_y);

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
	depth_image_sub = it.subscribe("depth_image", 1, &ConeLocatorNode::depthCallback, this);
	cone_location_pub = nh.advertise<geometry_msgs::Point>("cone_location", 1);
}

void ConeLocatorNode::depthCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		depth_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e) {
  		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;}
	return;
}

void ConeLocatorNode::locationCallback(const sensor_msgs::ImageConstPtr& msg){
	geometry_msgs::Point coords;

	if (!depth_image_ptr) return;

	Mat bw_image;
	try{
		bw_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
	}
	catch (cv_bridge::Exception& e) {
  		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


	int R = bw_image.rows, C = bw_image.cols;
	int area, high_x, low_x, high_y, low_y;
	area = high_x = high_y = 0;
	low_x, low_y = bw_image.cols;

	const int WHITE = 0;

	for(int r = 0; r < R; r += 5){
		for(int c = 0; c < C; c+= 5){
			if ((int)bw_image.at<uchar>(r,c) == WHITE){
				myfloodFill(bw_image, r, c, area, high_x, low_x, high_y, low_y);
				if (area > R*C*.1) goto found;
				else{
					area = high_x = high_y = 0;
					low_x, low_y = bw_image.cols;
				}
			}}}


	coords.x = bw_image.cols / 2;
	coords.y = bw_image.rows / 2;
	coords.z = 0;
	cone_location_pub.publish(coords);

	found:
	coords.x = (high_x - low_x) / 2;
	coords.y = (high_y - low_y) / 2;
	coords.z = 0;
	cone_location_pub.publish(coords);
}



// Not sure if we need a replace here if it's going to be pure white
void myfloodFill(Mat& image, int r, int c, int& area, int& high_x, int& low_x, int& high_y, int& low_y) {
	const int WHITE = 0, BLACK = 255;
	if (r < 0 || c < 0 || r >= image.rows || c >= image.cols) return;
	if ((int)image.at<uchar>(r,c) == BLACK) return;

	area += 1;
	if (r < low_x) low_x = r;
	if (r > high_x) high_x = r;
	if (c < low_y) low_y = c;
	if (c > high_y) high_y = c;

	myfloodFill(image, r+1, c, area, high_x, low_x, high_y, low_y);
	myfloodFill(image, r-1, c, area, high_x, low_x, high_y, low_y);
	myfloodFill(image, r, c+1, area, high_x, low_x, high_y, low_y);
	myfloodFill(image, r, c-1, area, high_x, low_x, high_y, low_y);
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "cone_locator_node");
	ConeLocatorNode node;
	ros::spin();
	return 0; 
}





