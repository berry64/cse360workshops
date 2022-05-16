import sys
import time
import math
import networkx as nx
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

import socket
import time

IP_ADDRESS = '192.168.0.207'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

positions = {}
rotations = {}



# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

g = nx.Graph()
g.add_node(1, x=5.30, y=-0.54)
g.add_node(2, x=3.88, y=-1.64)
g.add_node(3, x=5.09, y=2.00)
g.add_node(4, x=2.42, y=0.78)
g.add_node(5, x=2.98, y=3.58)
g.add_node(6, x=1.42, y=-0.54)
g.add_node(7, x=0.42, y=-1.47)
g.add_node(8, x=0.09, y=1.17)
g.add_node(9, x=-0.46, y=0.05)
g.add_node(10, x=-0.94, y=-0.46)
g.add_node(11, x=-1.13, y=1.55)
g.add_node(12, x=-1.75, y=-3.18)
g.add_node(13, x=-2.35, y=-0.25)
g.add_node(14, x=-2.29, y=1.38)
g.add_node(15, x=-4.09, y=-0.30)

edges = [(1,2),(1,3)
    ,(2,3),(2,7),(2,6)
    ,(3,5)
    ,(4,5),(4,6)
    ,(5,8)
    ,(6,7)
    ,(7,10),(7,12)
    ,(8,9),(8,11)
    ,(9,10)
    ,(10,13)
    ,(11,14)
    ,(12,13)
    ,(13,14),(13,15)]

for (a,b) in edges:
    dx = g.nodes[a]['x'] - g.nodes[b]['x']
    dy = g.nodes[a]['y'] - g.nodes[b]['y']
    wt = math.sqrt(dx*dx + dy*dy)
    g.add_weighted_edges_from([(a,b,wt)])
path = nx.shortest_path(g, source=3, target=15, weight='weight')

curtarget = 0
target = [g.nodes[path[0]]['x'], g.nodes[path[0]]['y']]

if __name__ == "__main__":
    clientAddress = "192.168.0.2"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 7
    kv = 1700
    kw = 1200

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    t0 = time.time()
    while is_running:
        if robot_id in positions:

            # last position
            #print('Position', positions[robot_id])

            #position = positions[robot_id]
            rot = math.radians(rotations[robot_id])
            curpos = [positions[robot_id][0], positions[robot_id][1]]
            dx = target[0] - curpos[0]
            dy = target[1] - curpos[1]

            if abs(dx) < 0.3 and abs(dy) < 0.3:
                curtarget += 1
                target = [g.nodes[path[curtarget]]['x'], g.nodes[path[curtarget]]['y']]
                print("Proceeding to waypoint %d at (%f, %f)"%(curtarget, target[0], target[1]))

            #print('G-P: (%f,%f)\n p-b:(%f,%f)\n dx,dy:(%f,%f)'%(target[0]-curpos[0], target[1]-curpos[1], ((curpos[0]-b[0])/math.pow(abs(curpos[0]-b[0]),3)), ((curpos[1]-b[1])/math.pow(abs(curpos[1]-b[1]),3)), dx,dy))

            targetrot = math.atan2(dy,dx)
            #print('TargetRot: ' + str(curwaypt)+ ',' + str(targetrot) + 'dx,dy: ' + str(dx) + "," +str(dy))

            omega = kw * math.atan2(math.sin(targetrot - rot), math.cos(targetrot - rot))
            
            v = kv * (math.sqrt(dx*dx+dy*dy))

            u = np.array([v - omega, v + omega])
            u[u > 1500] = 1500
            u[u < -1500] = -1500

            # Send control input to the motors
            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
            #print(command)
            s.send(command.encode('utf-8'))
            time.sleep(.05)

s.shutdown(2)
s.close()