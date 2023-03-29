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

# P controller
K1 = 1000
K2 = 7000

not_there = True

try:
    start = time.time()
    t=0
    while not_there:
        file = open("Part3.txt", "a+")
        if robot_id in positions:
            now = time.time() - start
            x = (-2)+(cos((now/5)))
            y = (-1)+(sin(now/5))
            # current position
            xy = np.array([positions[robot_id][0], positions[robot_id][1]])
            print('Current position', xy)
            # current rotation
            theta = rotations[robot_id] * pi/180

            # error
            errorx = x - xy[0]
            errory = y - xy[1]
            print('Error x', errorx, 'Error y', errory)

            # angle to destination
            alpha = atan2(errorx, errory)

            # angle to destination in robot frame
            diff = alpha - theta
            errorw = degrees(atan2(np.sin(diff), np.cos(diff)))

            time_elapsed = time.time() - start
            file.write(str(time_elapsed) + " " + str(x) + " " + str(y) + "\n")

            v = K1*(sqrt(errorx**2 + errory**2))
            w = K2 * errorw

            u = np.array([v - w, v + w])
            u[u > 1500.] = 1500.
            u[u < -1500.] = -1500.
            
            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
            s.send(command.encode('utf-8'))
            print(command)
            time.sleep(0.1)

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