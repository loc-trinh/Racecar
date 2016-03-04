#!/usr/bin/python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

pub = rospy.Publisher('cone_location', PoseArray, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(10) # 10hz

msg=PoseArray()

robot = Pose()
robot.position.x=0
robot.position.y=1

cone1= Pose()
cone1.position.x=2
cone1.position.y=0

cone2= Pose()
cone2.position.x=4
cone2.position.y=0

msg.poses=[robot,cone1,cone2]

while not rospy.is_shutdown():
   pub.publish(msg)
   r.sleep()