#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import nav_msgs.msg
import math

class GlobalPlanner:
	def __init__(self):
		rospy.Subscriber("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.planCallback)
		self.plan_pub = rospy.Publisher("global_plan", nav_msgs.msg.Path, queue_size=1)
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)

		self.plan = nav_msgs.msg.Path()
		self.plan.header.stamp = rospy.Time.now()
		self.plan.header.frame_id = "odom";

	def planCallback(self, goal):
		try:
			transform_ob = self.tfBuffer.lookup_transform('base_link', goal.header.frame_id, rospy.Time(0), rospy.Duration(1))
			transform_bo = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(1))
		except:
			print "TF ERROR!"
			return

		start = geometry_msgs.msg.PoseStamped()
		start.header.stamp = rospy.Time.now()
		start.header.frame_id = 'base_link'
		start.pose.position.x = start.pose.position.y = 0
		start.pose.orientation.x = start.pose.orientation.y = start.pose.orientation.z = 0
		start.pose.orientation.z = 1
		end = tf2_geometry_msgs.do_transform_pose(goal, transform_ob)

		#Creating plan
		step = 8
		points = []
		prev_point = start
		dx = end.pose.position.x - start.pose.position.x
		dy = end.pose.position.y - start.pose.position.y
		if abs(dy) < 1:
			points.append(tf2_geometry_msgs.do_transform_pose(start, transform_bo))
			for i in range(step):
				point = end
				point.pose.position.x = prev_point.pose.position.x + 1./step * dx
				point.pose.position.y = prev_point.pose.position.y + 1./step * dy
				prev_point = point
				points.append(tf2_geometry_msgs.do_transform_pose(point, transform_bo))
			points.append(tf2_geometry_msgs.do_transform_pose(end, transform_bo))
		else:
			sign = 1 if dy > 0 else -1
			det = math.log(.001) - math.log(sign * end.pose.position.y)
			A = end.pose.position.x/det
			B = end.pose.position.x*math.log(.001)/det
			points.append(tf2_geometry_msgs.do_transform_pose(start, transform_bo))
			for i in range(1,step+1):
				point = end
				point.pose.position.y = sign * i*dy/float(step)
				point.pose.position.x = -A * math.log(point.pose.position.y) + B
				point.pose.position.y *= sign
				points.append(tf2_geometry_msgs.do_transform_pose(point, transform_bo))
			points.append(tf2_geometry_msgs.do_transform_pose(end, transform_bo))

		self.plan.poses = points
		self.plan_pub.publish(self.plan)

if __name__ == "__main__":
	rospy.init_node("global_planner")
	node = GlobalPlanner()
	rospy.spin()