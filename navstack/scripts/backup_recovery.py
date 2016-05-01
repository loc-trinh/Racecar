#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import tf
import math

class BackupRecovery:
    cGoal = None
    def __init__(self):
        #Class members
        self.grid = None
        self.confident = 0
        self.topic_goal_in = "/plan_executor/goal_out"
        self.base_frame = "base_link"
        self.look_ahead = 0.75


        self.base_frame = rospy.get_param('~base_frame', self.base_frame)

        #Pubs & Subs
        rospy.Subscriber("/costmap_base/costmap/costmap", OccupancyGrid, self.costmap_callback)
        rospy.Subscriber("/costmap_base/costmap/costmap_updates", OccupancyGridUpdate, self.costmap_update_callback)
        rospy.Subscriber(self.topic_goal_in, PoseStamped, self.new_dest_callback)
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/nav", AckermannDriveStamped, queue_size=1)

        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

    def get_cell_range(self, pt1, pt2, meta):
        #1 - Convert points to map coords
        pt1[0] -= meta.origin.position.x;
        pt1[1] -= meta.origin.position.y;
        pt2[0] -= meta.origin.position.x;
        pt2[1] -= meta.origin.position.y;

        #2 - convert coords to rows and cols
        row_start  = int(pt1[0]*(1/meta.resolution));
        row_end = int(pt2[0]*(1/meta.resolution));
        col_start  = int(pt1[1]*(1/meta.resolution));
        col_end = int(pt2[1]*(1/meta.resolution));

        cells = []
        for col in range(col_start, col_end):
            for row in range(row_start, row_end):
                cells.append(self.getIndex(col,row));
        return cells

    def new_dest_callback(self,data):
        self.cGoal = data;

    def count(self,grid,cells):
        unknown = 0
        empty = 0
        full = 0;
        for cell in cells:
            if grid[cell] == -1:
                unknown+=1;
            elif grid[cell] == 0: 
                empty +=1;
            else:
                full += 1
        return (unknown, empty, full)

    def check_if_stuck(self):
        if self.grid == None:
            print "Waiting for Initial OCC Grid..."
            return

        #get target in base_link
        if(self.cGoal == None):
            rospy.loginfo("Warning: No goal")
            return

        self.cGoal.header.stamp = self.listener.getLatestCommonTime(self.base_frame,self.cGoal.header.frame_id)
        dest = self.listener.transformPose(self.base_frame, self.cGoal)

        #Generate points in direction:
        step_dist = 0.1;
        prev_point = (0,0)
        deltax = dest.pose.position.x - 0
        deltay = dest.pose.position.y - 0
        vnorm = math.sqrt(deltax**2 + deltay**2)
        dx = deltax / vnorm
        dy = deltay / vnorm
        d = 0;
        points = [(0,0)];
        while (d < self.look_ahead):
            x = points[-1][0] + step_dist * dx;
            y = points[-1][1] + step_dist * dy;
            points.append((x,y))
            d+= step_dist

        #Generate list of cells involved
        cells = set()
        for (x,y) in points:
            cells.add(self.positionToIndex(x,y))

        
        #Check for collision
        collide = False
        rospy.loginfo(cells)
        for cell in cells:
            if self.grid.data[cell] > 0:
                collide = True

        if collide:
            self.perform_backup_move();

    def perform_backup_move(self):
        rospy.loginfo("Performing Backup Recovery!")
        stopmsg = AckermannDriveStamped()
        stopmsg.header.stamp = rospy.Time.now()
        stopmsg.header.frame_id = "base_link"
        stopmsg.drive.speed = 0
        stopmsg.drive.acceleration = 100
        stopmsg.drive.steering_angle = 0

        bkpmsg = AckermannDriveStamped()
        bkpmsg.header.stamp = rospy.Time.now()
        bkpmsg.header.frame_id = "base_link"
        bkpmsg.drive.speed = -0.75
        bkpmsg.drive.acceleration = 1
        bkpmsg.drive.steering_angle = 0.3

        for i in range(1,25):
            stopmsg.header.stamp = rospy.Time.now()
            self.drive_pub.publish(stopmsg);
            rospy.sleep(.01)

        for i in range(1,50):
            bkpmsg.header.stamp = rospy.Time.now()
            self.drive_pub.publish(bkpmsg);
            rospy.sleep(.01)

        for i in range(1,10):
            stopmsg.header.stamp = rospy.Time.now()
            self.drive_pub.publish(stopmsg);
            rospy.sleep(.01)

    def positionToIndex(self,x,y):
        x -= self.grid.info.origin.position.x
        y -= self.grid.info.origin.position.y
        x = int(x*(1/self.grid.info.resolution));
        y  = int(y*(1/self.grid.info.resolution));
        sx = self.grid.info.width;
        return int(round(y * sx + x));

    def getIndex(self,x,y):
        sx = self.grid.info.width;
        return int(round(y * sx + x));

    def costmap_update_callback(self,data):
        if self.grid == None:
            print "Waiting for Initial OCC Grid..."
            return
        i = 0;
        for y in range(data.y, data.y+data.height):
            for x in range(data.x, data.x+data.width):
                self.grid.data[self.getIndex(x,y)] = data.data[i];
                i+=1

    def costmap_callback(self, data):
        self.grid = data;
        self.grid.data = list(self.grid.data)

if __name__ == "__main__":
    rospy.init_node("backup_recovery")
    backup = BackupRecovery();

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        backup.check_if_stuck();
        rate.sleep()