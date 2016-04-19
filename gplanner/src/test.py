import matplotlib.pyplot as plt
from math import log

start = (.01, 0)
end = (20,-20)

sign = 1
if end[1] > 0:
	sign = -1

end = (abs(end[1]),end[0])



det = log(start[0]) - log(end[0])
a = (start[1] - end[1])/float(det)
b = (-log(end[0])*start[1] + log(start[0])*end[1])/float(det)

path = []
path.append(start)
for i in range(1,100):
	dx = (end[0]-start[0])/float(100)
	x = 0+i*dx
	y = log(x)*a + b
	path.append((x,y))
path.append(end)

x = [sign * i[0] for i in path]
y = [i[1] for i in path]
plt.plot(x,y)
plt.show()
