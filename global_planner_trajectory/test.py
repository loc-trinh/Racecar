import matplotlib.pyplot as plt

pt1 = (0,0)
pt3 = (5,5)
pt2 = (.5,3)

a11 = pt1[0]**2
a12 = pt1[0]
a13 = 1
a21 = pt2[0]**2
a22 = pt2[0]
a23 = 1
a31 = pt3[0]**2
a32 = pt3[0]
a33 = 1

det = a11*a22*a33 + a12*a23*a31 + a13*a21*a32 - a31*a22*a13 - a32*a23*a11 - a33*a21*a12

r11 = (a22*a33 - a32*a23) * 1/float(det)
r12 = (a13*a32 - a33*a12) * 1/float(det)
r13 = (a12*a23 - a22*a13) * 1/float(det)
r21 = (a23*a31 - a33*a21) * 1/float(det)
r22 = (a11*a33 - a31*a13) * 1/float(det)
r23 = (a13*a21 - a23*a11) * 1/float(det)
r31 = (a21*a32 - a31*a22) * 1/float(det)
r32 = (a12*a31 - a32*a11) * 1/float(det)
r33 = (a11*a22 - a21*a12) * 1/float(det)

a = r11*pt1[1] + r12*pt2[1] + r13*pt3[1]
b = r21*pt1[1] + r22*pt2[1] + r23*pt3[1]
c = r31*pt1[1] + r32*pt2[1] + r33*pt3[1]

start = pt1
end = pt3
path = []

path.append(start)
for i in range(20):
  dx = 1/float(20) * (end[0]-start[0])
  path.append((dx*i + start[0], a * (dx*i+start[0])**2 + b * (dx*i+start[0]) + c))
path.append(end)

x = [i[0] for i in path]
y = [i[1] for i in path]

plt.plot(x,y)
plt.show()



