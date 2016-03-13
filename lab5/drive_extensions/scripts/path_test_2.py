#!/usr/bin/python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PointStamped

pub = rospy.Publisher('destination_point', PointStamped, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(10)

msg = PointStamped()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id='base_link'

dest_point = Point()
dest_point.x = 5
dest_point.y = 1
dest_point.z = 0
msg.point = dest_point

while not rospy.is_shutdown():
   pub.publish(msg)
   r.sleep()