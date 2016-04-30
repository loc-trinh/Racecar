map = [
0,0,0,0,0,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0,
99,99,99,99,99,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0,
0,0,0,0,0,0,0,0,0,99,0,0]

import math
class Map(object):
    def __init__(self, iMap):
        self.occupancyArray= iMap
        self.width= 12
        self.height= 12
        self.resolution = .05

    def map_data(self,x,y):
        return self.occupancyArray[int(x)*self.width+int(y)]

    def map_valid(self,x,y):
        return (x<self.width and x>=0 and y<self.height and y>=0)

    def calc_range(self, robot_x,robot_y,robot_a,max_range):
        x0, y0 = robot_x, robot_y
        x1, y1 = (robot_x + max_range * math.cos(robot_a), 
                 robot_y + max_range * math.sin(robot_a))

        if abs(y1-y0) > abs(x1-x0):
            steep = True
        else:
            steep = False

        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1

        deltax = abs(x1-x0)
        deltay = abs(y1-y0)
        error = 0
        deltaerr = deltay

        x = x0
        y = y0

        if x0 < x1:
            xstep = 1
        else:
            xstep = -1
        if y0 < y1:
            ystep = 1
        else:
            ystep = -1

        if steep:
            if not self.map_valid(y, x) or self.map_data(y,x) == -1 or self.map_data(y,x) > 50:
                return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)))
        else:
            if not self.map_valid(x, y) or self.map_data(x,y) == -1 or self.map_data(x,y) > 50:
                return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)))

        while x != (x1 + xstep*1):
            x += xstep
            error += deltaerr
            if 2*error >= deltax:
                y += ystep
                error -= deltax

            if steep:
                if not self.map_valid(y, x) or self.map_data(y,x) == -1 or self.map_data(y,x) > 50:
                	print "RETURNING", y, x
                	return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)))
            else:
                if not self.map_valid(x, y) or self.map_data(x,y) == -1 or self.map_data(x,y) > 50:
                	print "RETURNING", x, y
                	return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)))

        return max_range

m = Map(map)
print m.calc_range(0,0,math.pi/2,100)