%pylab inline

# Perfect sensor
def sense(x):
    return x

def simulate(Δt, x, u):
    x += Δt * u
    return x

def control_q1(t, y):
    ### WRITE YOUR CONTROL POLICY HERE:
    ux = 3.464 * sin(t-pi) + cos(t-pi)
    uy = 2 * cos(t-pi) + sin(t-pi)
    return array([ux, uy])

def control_q2(t,y):
    ux = 5*cos(5*t)*cos(t)-sin(t)*(sin(5*t)+2)
    uy = 5*cos(5*t)*sin(t)+cos(t)*(sin(5*t)+2)
    return array([ux,uy])

kp = 5
ki = 1
ex = 0
ey = 0
def control_q2_pi(t,y):
    dx = (sin(5*t)+2)*cos(t)
    dy = (sin(5*t)+2)*sin(t)
    global ex, ey
    ex += ki*(dx-y[0])
    ey += ki*(dy-y[1])
    ux = kp*(dx-y[0])+ex
    uy = kp*(dy-y[1])+ey
    return array([ux,uy])

def control_helix(t,y):
    ux = -sin(t)
    uy = cos(t)
    uz = 0.1
    return array([ux, uy, uz])


# Initial conditions
x = array([0., 2.])
x_log = [copy(x)]

x_3d = array([0., 0., 0.])
x_log_3d = [copy(x_3d)]

for t in time:
    y = sense(x_3d)
    u = control_helix(t,y)
    x_3d = simulate(Δt, x_3d, u)
    x_log_3d.append(copy(x_3d))

    y = sense(x)
    u = control_q2_pi(t, y)
    u += [0, -0.1]
    #u += [randint(0,200)/100 - 1, randint(0,200)/100 - 1]
    x = simulate(Δt, x, u)
    x_log.append(copy(x))
    
x_log = array(x_log)
x_log_3d = array(x_log_3d)

#grid()
#plot(x_log[:,0], x_log[:,1])
import matplotlib.pyplot as plt
ax = plt.axes(projection='3d')
ax.plot3D(x_log_3d[:,0], x_log_3d[:,1], x_log_3d[:,2], 'gray')
