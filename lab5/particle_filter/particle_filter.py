

def MCL(iMap, particles, odom, sense):
    xBar=[]
    xReal=[]
    for m in range(0,len(particles)):
        x=motion_update(odom, particles[m])
        weight= sensor_update(iMap,sense, particles[m])
        xBar.append((x,weight))
    #Normalize weights
    total=0
    for pair in xbar:
        total+=pair[1]
    for pair in xbar:
        pair[1]=pair[1]/float(total)
    #Resample
    for i in range(0,len(particles)):
        xReal.append(sample(xBar))
    return xReal

def motion_update(odom, particle):
    pass

def sensor_update(iMap,sense, particle):
    pass

def sample(xBar):
    pass


