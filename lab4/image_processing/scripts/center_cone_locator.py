#!/usr/bin/python
import rospy
from std_msgs.msg import Float32
import threading

class CenterConeLocator:
    def __init__(self):
        self.left_cone_sub = rospy.Subscriber("left/cone_location", Float32, self.left_callback)
        self.right_cone_sub = rospy.Subscriber("right/cone_location", Float32, self.right_callback)
        self.cone_pub = rospy.Publisher("cone_location", Float32, queue_size=5)
        self.lock = threading.Lock()
        self.left_cone_position = None


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
        self.cone_pub.publish(avg_theta_msg)
            

if __name__=="__main__":
    rospy.init_node("ConeDetector")
    node = CenterConeLocator()
    rospy.spin()

