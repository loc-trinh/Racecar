#!/usr/bin/python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class RGB_to_black_whit_RGB:
	def __init__(self):
		self.image_pub=rospy.Publisher("bw_image", Image)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("image_rect_color", Image, self.callback)
	def callback(self, data):
		cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		#convert to HSV
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		#set the ranges we are interested in
		low_orange_threshold=np.array([0, 52, 184], dtype=np.uint8)
		high_orange_threshold=np.array([255,255,255], dtype=np.uint8)
		#threshold the isolate the orange colors
		mask = cv2.inRange(hsv, lower_blue, upper_blue)
		#isolate the orange colors in the original image
		res = cv2.bitwise_and(frame,frame, mask= mask)

		#display all three images
		cv2.imshow('frame',frame)
    	#cv2.imshow("mask",mask)
    	#cv2.imshow('res',res)
    	cv2.waitKey(3)#
		ros_img=self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		self.image_pub.publish(ros_img)
def main(args):
	ic=RGB_to_black_whit_RGB()
	rospy.init_node('rgb_to_black_whute_rgb')
	try:
		rospy.spin()
	except KeyboardInterrupt:

		print ("Shutting down")
	cv2.destroyAllWindows()
if __name__ == '__main__':
	main(sys.argv)		

