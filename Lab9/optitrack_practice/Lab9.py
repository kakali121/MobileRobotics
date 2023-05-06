import sys
import time
import numpy as np
import networkx as nx
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
route = []

n, m = 6, 8

def set_obstacles(environment):
    environment[0][3] = 1
    environment[0][5] = 1
    environment[1][4] = 1
    environment[2][0] = 1
    environment[3][7] = 1
    environment[4][2] = 1
    environment[4][4] = 1
    environment[5][5] = 1
    environment[5][2] = 1
    return environment


def get_path(start, end):
    environment = list()
    for i in range(n):
        environment_row = list()
        for j in range(m):
            environment_row.append([(-1.0+0.7*j), (1-0.7*i)])
        environment.append(environment_row)
    # print(environment)

    # Setting obstacles
    environment = set_obstacles(environment)

    # Creating graph
    G = nx.grid_2d_graph(n, m)

    # Delete nodes with obstacles
    for i in range(n):
        for j in range(m):
            if environment[i][j] == 1:  
                G.remove_node((i,j))
    # print(G.nodes())

    bfs = list(nx.bfs_edges(G, start))
    # print(bfs)
    # Pick the last element and iterate through its predecessors
    path = [end]
    current = end

    # iterate through its predecessors until finding source node
    while current != start:
        # Predecesors of the current node        
        for pre in bfs.predecessors(current):
            current = pre
        # add the predecessor to the path
        path.append(pre)

    # The current path starts in the goal node and ends at the start node. So we invert it
    path = path[::-1]

    for i in range(len(path)):
        route.append(environment[(path[i][0])][(path[i][1])])
    # print(path)
    # print(route)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = (rotz)


if __name__ == "__main__":
    clientAddress = "192.168.0.23"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 321

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

    start = (5, 0)
    end = (0, 7)
    
    get_path(start, end)
    # print(route)

    # P controller
    K1 = 2500
    K2 = 100

    not_there = True

    try:
        start = time.time()
        end = time.time()
        while not_there:
            if robot_id in positions:
                # current position
                xy = np.array(positions[robot_id])
                # current rotation
                theta = rotations[robot_id] * np.pi/180
                # print('Robot position', xy)
                # print('Robot rotation', theta)

                cur_node = 0

                ## Follow the path
                if abs(route[cur_node][0]-positions[robot_id][0])<0.5 and abs(route[cur_node][1]-positions[robot_id][1])<0.5:
                    print('Hit a waypoint:', route[cur_node])
                    cur_node+=1
                    if cur_node == len(route):
                        not_there = False
                        break

                # Calculate the angle to the next waypoint in the route
                desired_x = route[cur_node][0]
                desired_y = route[cur_node][1]

                error_x = desired_x - xy[0]
                error_y = desired_y - xy[1]
                # print('Error', error_x, error_y)

                alpha = np.arctan2(error_y, error_x)
                # angle to destination in robot frame
                diff = alpha - theta
                # print('Alpha', alpha)
                error_w = np.arctan2(np.sin(diff), np.cos(diff))
                # print('Error w', error_w)
                
                v = K1*(np.sqrt(error_x**2 + error_y**2))
                w = K2 * error_w

                u = np.array([v - w, v + w])
                u[u > 1500.] = 1500.
                u[u < -1500.] = -1500.
                
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                print(command)
                time.sleep(0.1)

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