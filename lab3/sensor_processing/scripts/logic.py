import numpy as np
x = np.array([-2,-1,0,1,2])
y = np.array([4,3,2,1,0])*-1
origin = [0,0]

m, b = np.polyfit(x,y,1)
norm = [-m, 1]
print "Norm: ", norm
print "Distance: ", abs(np.dot(norm, origin)-b)/np.linalg.norm(norm)
angle = np.math.atan2(np.cross(norm, [0,1]), np.dot(norm, [0,1]))
angle = np.math.pi/2 - angle if angle > 0 else -(np.math.pi/2 + angle)
print "Angle: ", angle

