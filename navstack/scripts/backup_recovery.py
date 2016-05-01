#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from ackermann_msgs.msg import AckermannDriveStamped


class BackupRecovery:
    def __init__(self):
        #Class members
        self.grid = None
        self.confident = 0

        #Pubs & Subs
        rospy.Subscriber("/costmap_base/costmap/costmap", OccupancyGrid, self.costmap_callback)
        rospy.Subscriber("/costmap_base/costmap/costmap_updates", OccupancyGridUpdate, self.costmap_update_callback)
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/nav", AckermannDriveStamped, queue_size=1)

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

        cells = self.get_cell_range([-.5,0],[.5,.5],self.grid.info)
        unknown, empty, full = self.count(self.grid.data, cells)

        if full > 10:
            self.confident += 1

        if self.confident > 5:
            self.confident = 0
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"
            msg.drive.speed = -5
            msg.drive.steering_angle = 0
            self.drive_pub.publish(msg)

    def getIndex(self,x,y):
        sx = self.grid.info.width;
        return y * sx + x;

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