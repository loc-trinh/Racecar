#!/usr/bin/python

###############
##
## Freespace Planner
## 
## A really basic goal-generator based on 
## available free-space up ahead.
##
###############

# Python Includes
import numpy as np
import math

# ROS Includes
import rospy
import tf

# ROS messages
#from geometry_msgs.msg import PoseArray, Pose, PointStamped
from nav_msgs.msg import OccupancyGrid

class FreespacePlanner:
    def __init__(self):
        #Default settings
        self.kfactor = 0.6;
        self.topic_occgrid = "/move_base/global_costmap/costmap";
        self.topic_output = "move_goal";
        self.base_frame = "base_link"
        self.map_frame = "odom";

        #Load Settings
        self.kfactor = rospy.get_param('~kfactor', self.kfactor)
        self.topic_occgrid = rospy.get_param('~topic_input', self.topic_occgrid)
        self.topic_output = rospy.get_param('~topic_output', self.topic_output)
        self.base_frame = rospy.get_param('~base_frame', self.base_frame)
        self.map_frame = rospy.get_param('~map_frame', self.map_frame)

        #Pubs and Subs
        #self.goal_pub = rospy.Publisher(self.topic_output, PoseArray, queue_size=10)
        rospy.Subscriber(self.topic_input, OccupancyGrid, self.costmap_callback)

        #Setup Data Structures
        self.cone_array = PoseArray();
        self.cone_array.header.frame_id = self.map_frame;

        #Setup TF
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

        rospy.loginfo("FreeSpace Planner node loaded")


    def costmap_callback(self, data):
        print "==============="

        data.header.stamp = self.listener.getLatestCommonTime(self.map_frame,data.header.frame_id)
        con_loc = self.listener.transformPoint(self.base_frame, data)

        width = data.info.width;
        height = data.info.height;
        resolution 


        # Filter out bad points
        if data.point.x == 0 and data.point.y == 0:
            print "No Cone"
            return;
        elif data.point.x > 5 and data.point.y > 5:
            print "Filtering out far cone"
            return;
        # Get point and transform into odom frame
        data.header.stamp = self.listener.getLatestCommonTime(self.map_frame,data.header.frame_id)
        con_loc = self.listener.transformPoint(self.map_frame, data)

        #Prune list
        i = 0
        j = 0
        removeList = [];
        for i in range(len(self.cone_array.poses)-1):
            for j in range(i+1, len(self.cone_array.poses)-1):
                if math.sqrt( (self.cone_array.poses[i].position.x - self.cone_array.poses[j].position.x)**2 + (self.cone_array.poses[i].position.y - self.cone_array.poses[j].position.y)**2 ) < self.kfactor:
                    removeList.append(self.cone_array.poses[j]);

        self.cone_array.poses = [n for n in self.cone_array.poses if n not in removeList]


        # Compare against existing cones
        matched = False
        print "Ary Length = %f" % len(self.cone_array.poses)
        for cone in self.cone_array.poses:
            if math.sqrt( (con_loc.point.x - cone.position.x)**2 + (con_loc.point.y - cone.position.y)**2 ) < self.kfactor:
                print "Cone Match!"
                cone.position.x = con_loc.point.x;
                cone.position.y = con_loc.point.y;
                matched = True
                break

        # If no match, add to cone list
        if not matched:
            print "Adding Cone at (%f,%f)!" % (con_loc.point.x, con_loc.point.y)
            cone = Pose();
            cone.position.x = con_loc.point.x;
            cone.position.y = con_loc.point.y;
            self.cone_array.poses.append(cone);


        ## Publish array. Note: Publishing in odom
        self.cone_array.header.stamp = rospy.Time.now();
        self.publisher.publish(self.cone_array)



if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node("freespace_planner")

    FreespacePlanner();

    rospy.spin();
    
    # enter the ROS main loop
    '''
    rate = rospy.Rate(estimator.rate)
    while not rospy.is_shutdown():
        rospy.loginfo(hello_str)


        pub.publish(hello_str)
        rate.sleep()
    '''


