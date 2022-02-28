def cs(arr,t): #compute_spline
    return arr[0]+arr[1]*t+arr[2]*t*t+arr[3]*t*t*t

splinex = array([[-5,0,2.8125,-0.46875], [10,0,-1.333,0.259], [5,-1,0,0.037], [3,0,-1,0.222],[0,0,0.833,-0.167],[3,0.5,-0.333,0.093],[4,1,1.000,-0.259]])
spliney = array([[-7,0,0.000,0.000],[-7,0,22.000,-28.000],[-5,1,0.000,-0.037],[-3,0,0,0.185],[2,5,-0.667,-0.222],[5,-5,2.000,-0.185],[3,2,1.000,-0.296]])

def path_spline(t):
    if t<4:
      ux = cs(splinex[0],t)
      if t<3.5:
        uy = cs(spliney[0],t)
      else:
        uy = cs(spliney[1],t-3.5)
    elif t<7:
      ux = cs(splinex[1],t-4)
      uy = cs(spliney[2],t-4)
    elif t<10:
      ux = cs(splinex[2],t-7)
      uy = cs(spliney[3],t-7)
    elif t<13:
      ux = cs(splinex[3],t-10)
      uy = cs(spliney[4],t-10)
    elif t<16:
      ux = cs(splinex[4],t-13)
      uy = cs(spliney[5],t-13)
    elif t<19:
      ux = cs(splinex[5],t-16)
      uy = cs(spliney[6],t-16)
    else:
      ux = cs(splinex[6],t-19)
      uy = 10
    return array([ux,uy])

#plot spline
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
pp1 = Rectangle((4,-1),2,10)
pp2 = Rectangle((0.5,2),2,3)
pp3 = Rectangle((-6,-5),15,1)
pp4 = Rectangle((8,0),2,5)
pp5 = Rectangle((-4,-3),2,8)
ax.add_patch(pp1)
ax.add_patch(pp2)
ax.add_patch(pp3)
ax.add_patch(pp4)
ax.add_patch(pp5)

time = linspace(0., 22, int(22.0/0.01) + 1)
path_sum = []
for t in time:
  path_sum.append(path_spline(t))

path_sum = array(path_sum)
print(path)
ax.plot(path_sum[:,0], path_sum[:,1])
