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

// For flood fill
#include <set>
#include <queue>

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

typedef struct Point{
	int x, y;
} myPoint;

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
	depth_image_sub = it.subscribe("/camera/depth/image_rect_color", 1, &ConeLocatorNode::depthCallback, this);
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

	//if (!depth_image_ptr) return;

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

	const int WHITE = 255;

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
	queue<myPoint> queue;
	set<int> has_seen;

	myPoint pt = {r,c};
	queue.push(pt);

	const int WHITE = 255, BLACK = 0;

	while (!queue.empty()){
		myPoint pt = queue.front();
		queue.pop();
		if (has_seen.find(pt.x*10000+pt.y) != has_seen.end()){
			has_seen.insert(pt.x*10000+pt.y);
		}else{
			continue;
		}

		if (pt.x < 0 || pt.y < 0 || pt.x >= image.rows || pt.y >= image.cols) continue;
		if ((int)image.at<uchar>(r,c) == BLACK) continue;
		area += 1;
		if (pt.x < low_x) low_x = pt.x;
		if (pt.x > high_x) high_x = pt.x;
		if (pt.y < low_y) low_y = pt.y;
		if (pt.y > high_y) high_y = pt.y;

		myPoint left = {pt.x-1, pt.y};
		myPoint right = {pt.x+1, pt.y};
		myPoint up = {pt.x, pt.y+1};
		myPoint down = {pt.x, pt.y-1};


		queue.push(up);
		queue.push(down);
		queue.push(left);
		queue.push(right);
		
	}
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "cone_locator_node");
	ConeLocatorNode node;
	ros::spin();
	return 0; 
}





