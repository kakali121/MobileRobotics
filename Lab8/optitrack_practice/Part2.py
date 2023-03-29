import sys
from math import *
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
import time

IP_ADDRESS = '192.168.0.212'

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


clientAddress = "192.168.0.25"
optitrackServerAddress = "192.168.0.4"
robot_id = 212

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

# destination
xy_dest = np.array([-2, 1])

# P controller
K1 = 2500
K2 = 100

not_there = True

try:
    start = time.time()
    while not_there:
        file = open("Part2.txt", "a+")
        if robot_id in positions:
            # current position
            xy = np.array([positions[robot_id][0], positions[robot_id][1]])
            print('Current position', xy)
            # current rotation
            theta = rotations[robot_id] * pi/180

            # error
            error = xy_dest - xy
            print('Error x', error[0], 'Error y', error[1])
            # distance to destination
            dist = np.linalg.norm(error)
            print('Distance to destination', dist)
            # angle to destination
            alpha = atan2(error[1], error[0])
            # angle to destination in robot frame
            diff = alpha - theta
            #print('Alpha', alpha, 'Theta', theta, 'Diff', diff)
            errorw = degrees(atan2(np.sin(diff), np.cos(diff)))
            print('Angle to destination', errorw)

            time_elapsed = time.time() - start
            file.write(str(time_elapsed) + " " + str(dist) + " " + str(diff) + "\n")

            v = K1*(sqrt(error[0]**2 + error[1]**2))
            w = K2 * errorw

            u = np.array([v - w, v + w])
            u[u > 1500.] = 1500.
            u[u < -1500.] = -1500.
            
            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
            s.send(command.encode('utf-8'))
            print(command)
            time.sleep(0.1)

            if abs(error[0]) < 0.1 and abs(error[1]) < 0.1:
                print("Done!")
                streaming_client.shutdown()
                not_there = False
                break

    file.close()
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    streaming_client.shutdown()

# Close the connection
s.shutdown(2)
s.close()