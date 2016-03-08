#!/usr/bin/python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped 
import numpy as np
import threading


class CenterConeLocator:
    def __init__(self):
        self.left_cone_sub = rospy.Subscriber("left/cone_location", Float32, self.left_callback)
        self.right_cone_sub = rospy.Subscriber("right/cone_location", Float32, self.right_callback)
        self.left_x_sub = rospy.Subscriber("left/x", Float32, self.left_x_callback)
        self.right_x_sub = rospy.Subscriber("right/x", Float32, self.right_x_callback)
        self.cone_pub = rospy.Publisher("cone_location", Float32, queue_size=5)
        self.conexy_pub = rospy.Publisher("cone_xy", PointStamped, queue_size=5)
        self.lock = threading.Lock()

        self.left_cone_position = None
        self.left_x = None
        self.theta = None

        self.stampedpoint=PointStamped()
        self.counter=0


    def left_callback(self, msg):
        with self.lock:
            self.left_cone_position = msg.data

    def right_callback(self,msg):
        with self.lock:
            left = self.left_cone_position

        if left == None:
            return

        right = msg.data
        avg_theta_msg = Float32()
        avg_theta_msg.data = (float(right) + float(left)) / 2.0
        self.theta = avg_theta_msg.data
        self.cone_pub.publish(avg_theta_msg)

    def left_x_callback(self, msg):
        with self.lock:
            self.left_x = msg.data

    def right_x_callback(self, msg):
        with self.lock:
            left_x = self.left_x

        if left_x == None or self.theta == None:
            return

        right_x = msg.data

        point = Point()
        point.x = (120*2.8)/(left_x-right_x)
        point.y = 0.0
        point.z = 0.0

        time=rospy.Time.now()
        self.counter+=1
        self.stampedpoint.header.seq=self.counter
        self.stampedpoint.header.frame_id="base_link"
        self.stampedpoint.header.stamp=time
        self.stampedpoint.point=point
        self.cone_xy.publish(self.stampedpoint)



if __name__=="__main__":
    rospy.init_node("ConeDetector")
    node = CenterConeLocator()
    rospy.spin()

