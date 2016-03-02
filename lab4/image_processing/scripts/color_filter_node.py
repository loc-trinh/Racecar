#!/usr/bin/python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def imageFilter():
    #roslib.load_manifest('my_package')
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    
    bridge = CvBridge()
    image_pub=rospy.Publisher("imagetopic", Image)
        #self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)
    rospy.init_node('image_filter')
    rate = rospy.Rate(10) # 10hz
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
    # Take each frame
        _, frame = cap.read()
    # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
        lower_orange = np.array([10, 74, 220], dtype=np.uint8)
        upper_orange = np.array([37,120,250], dtype=np.uint8)
    # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
    #print mask
    # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)
    #print res
        cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
        cv2.imshow('res',res)
        cv2.waitKey(1)
        image_to_pub=bridge.cv2_to_imgmsg(frame, "rgb8")
        image_pub.publish(image_to_pub)
if __name__ == '__main__':
    try:
        imageFilter()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()