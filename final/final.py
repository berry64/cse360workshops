from locale import currency
import sys
import time
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
from Motor import *
from Ultrasonic import *
ultrasonic = Ultrasonic()
PWM = Motor()

import time

positions = {}
rotations = {}    


"""
Function to get ultrasonic distance

PARAMS:
======
px, py, theta

RETURN:
=======
None if sensor reads obstacle too far away
else: obstacle points tuple (x,y) in world frame coordinates

"""
def detectObstacle(py, px, theta):
    data = ultrasonic.get_distance()
    # print('Data: ', data)
    distance = data/100 # Divide by 100 for CM
    if distance > 10:
        return None
    #print("Distance =", distance)
    obstacle = (px + distance * math.cos(theta),
                py + distance * math.sin(theta))

    return obstacle

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

target = [0,0]

if __name__ == "__main__":
    clientAddress = "192.168.0.210"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 10
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

    # store obstacles
    obstacles = []
    K = 0.5 # how far we want to be away from obstacle
    while is_running:
        if robot_id in positions:

            # last position
            #print('Position', positions[robot_id])

            #position = positions[robot_id]
            rot = math.radians(rotations[robot_id])
            curpos = [positions[robot_id][0], positions[robot_id][1]]
            obstacle = detectObstacle(curpos[0], curpos[1], rot)
            if obstacle is not None:
                obstacles.append(obstacle)

            dx = target[0] - curpos[0]
            dy = target[1] - curpos[1]
            # add obstacle vector
            for obstacle in obstacles:
                # Obstacle avoidance vector
                errpbx = curpos[0] - obstacle[0]
                errpby = curpos[1] - obstacle[1]
                # Calculate norm-3
                pbnorm3 = math.sqrt(errpbx**2 + errpby**2)**3
                # Divide by norm 3 and multiply by distance constant K
                errpbx = (errpbx / pbnorm3) * K
                errpby = (errpby / pbnorm3) * K

                # add obstacle avoidance vector
                dx = dx + errpbx
                dy = dy + errpby

            if(time.time() - t0 > 10):
                # NB: the rescue circle is centered at (5.43, 0)
                # Prolly change here? test tomorrow
                target = [0,0]
            
            targetrot = math.atan2(dy,dx)
            #print('TargetRot: ' + str(curwaypt)+ ',' + str(targetrot) + 'dx,dy: ' + str(dx) + "," +str(dy))

            omega = kw * math.atan2(math.sin(targetrot - rot), math.cos(targetrot - rot))
            
            v = kv * (math.sqrt(dx*dx+dy*dy))

            u = np.array([v - omega, v + omega])
            u[u > 1500] = 1500
            u[u < -1500] = -1500

            # Send control input to the motors
            PWM.setMotorModel(u[0],u[0],u[1],u[1])
            time.sleep(.05)