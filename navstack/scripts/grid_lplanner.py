#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf
import math

class GridLocalPlanner:
    goRight = True
    speed = (0.7,1.25,2)
    theta = (0.3, 0.2, 0.1, 0.5)
    def __init__(self):
        #Class members
        self.grid = None
        self.confident = 0
        self.topic_recovery = "/recovery/direction";
        self.base_frame = "base_link"
        self.look_ahead = 0.75

        #Cutoff regions
        self.x_space = 0.25
        self.y_space = 0.5

        self.base_frame = rospy.get_param('~base_frame', self.base_frame)

        #Pubs & Subs
        rospy.Subscriber("/costmap_base/costmap/costmap", OccupancyGrid, self.costmap_callback)
        rospy.Subscriber("/costmap_base/costmap/costmap_updates", OccupancyGridUpdate, self.costmap_update_callback)

        rospy.Subscriber(self.topic_recovery, Bool, self.recover_callback)

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/nav2", AckermannDriveStamped, queue_size=1)
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

        if row_start > row_end:
            (row_start, row_end) = self.flip(row_start, row_end)
        if col_start > col_end:
            (col_start, col_end) = self.flip(col_start, col_end)

        cells = []
        for col in range(col_start, col_end):
            for row in range(row_start, row_end):
                cells.append(self.getIndex(col,row));
        return cells

    def flip(self,a,b):
        return (b, a)

    def get_region(self, x,y,meta):
        if x == 0:
            x1 = -self.x_space
            x2 = self.x_space
        else:
            x1 = 2* x * self.x_space
            x2 = x * self.x_space

        y1 = y * self.y_space
        y2 = self.y_space + y*self.y_space

        return self.get_cell_range([x1,y1], [x2,y2], meta)

    def recover_callback(self,data):
        self.goRight = data.data;

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

    def update(self):
        if self.grid == None:
            print "Waiting for Initial OCC Grid..."
            return
        """       
        try: 
            self.cGoal.header.stamp = self.listener.getLatestCommonTime(self.base_frame,self.cGoal.header.frame_id)
            dest = self.listener.transformPose(self.base_frame, self.cGoal)
        except:
            print "TF Error!"
            return
        """


        #Figure out which cells are filled
        cells = [[False]*3,[False]*3,[False]*3]


        for i in range(0,3):
            for j in range(0,3):
                cell = self.get_region(i-1,j,self.grid.info)
                (unknown, empty, full) = self.count(self.grid.data, cell)
                cells[2-i][j] = full > 3
                print "(%d,%d) # filled = %d, cell[%d][%d] = %d" %(i,j,full,i,j, cells[i][j])

        #Is there a collision risk?
        collide = False
        collide = cells[1][0] or cells[1][1] or cells[1][2]
        print collide
        if not collide:
            return

        turnRight = self.goRight;

        #Is the turn direction deterministic?
        left_open = not (cells[0][0] or cells[0][1] or cells[0][2])
        right_open = not (cells[2][0] or cells[2][1] or cells[2][2])

        left_possible = not cells[0][0] or not cells[0][1] or not cells[0][2]
        right_possible = not cells[0][0] or not cells[0][1] or not cells[0][2]
        right=-1;
        left=1;

        print "======"
        print "left open: %s" % left_open
        print "Right open: %s" % right_open
        print "left possible: %s" % left_possible
        print "right possible: %s" % right_possible
        print "right Preference: %s" % turnRight

        if not left_open and right_open:
            self.drive(cells,right)
        elif left_open and not right_open:
            self.drive(cells,left)
        elif turnRight and right_possible:
            self.drive(cells,right)
        elif not turnRight and left_possible:
            self.drive(cells,left)
        elif right_possible:
            self.drive(cells,right)
        elif left_possible:
            self.drive(cells,left)
        else:
            self.drive(cells,right)


    def drive(self, cells, direction):
        xind = 1 - direction;

        #determine velocity
        if cells[1][0]:
            vel = self.speed[0]
        elif cells[1][1]:
            vel = self.speed[1]
        else:
            vel = self.speed[2]

        #determine angle
        if cells[xind][0]:
            theta = self.theta[0]
        elif cells[xind][1]:
            theta = self.theta[1]
        elif cells[xind][2]:
            theta = self.theta[2]
        else:
            theta = self.theta[3]

        theta *= direction

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.speed = vel
        msg.drive.acceleration = 16
        msg.drive.steering_angle = theta

        self.drive_pub.publish(msg);



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
    glp = GridLocalPlanner();

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        glp.update();
        rate.sleep()