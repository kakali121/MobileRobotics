import sys
import time
import rpc, struct
import numpy as np
import networkx as nx
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
import time
import pygame, io

IP_ADDRESS = '192.168.0.211'
omv_ip = "192.168.0.23"

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

positions = {}
rotations = {}
route = [[5.3, 3],
[5.3, 2],
[4.3, 2],
[4.3, 1],
[3.3, 1],
[2.3, 1],
[1.2999999999999998, 1],
[0.2999999999999998, 1],
[-0.7000000000000002, 1],
[-0.7000000000000002, 2],
[-0.7000000000000002, 3],
[-1.7000000000000002, 3],
[-2.7, 3],
[-3.7, 3]]
# kkl = [[5.5, 0], []]

n, m = 6, 10


def jpg_frame_buffer_cb(data):
    sys.stdout.flush()
    try:
        screen.blit(pygame.transform.scale(pygame.image.load(io.BytesIO(data), 'jpg'), (screen_w, screen_h)), (0, 0))
        pygame.display.update()
        clock.tick()
    except pygame.error: pass
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = (rotz)


def set_obstacles(environment):
    environment[0][3] = 1
    environment[0][5] = 1
    environment[1][4] = 1
    environment[2][0] = 1
    environment[3][7] = 1
    environment[4][2] = 1
    environment[4][4] = 1
    environment[4][8] = 1
    environment[4][9] = 1
    environment[5][5] = 1
    environment[5][2] = 1
    return environment


def get_path(start, end):
    environment = list()
    for i in range(n):
        environment_row = list()
        for j in range(m):
            environment_row.append([(5.3-j), (3-i)])
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

    bfs = nx.bfs_tree(G, start)
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


# color detection remote call for pink ballon
def exe_duck_detection(interface):
    time.sleep(0.5)
    result = interface.call("duck_detection")
    if result is not None and len(result):
        res = struct.unpack("<HHHH", result)
        print("cx:{} cy:{} w:{} h:{}".format(res[0], res[1], res[2], res[3]))
        if (res[0] == 0 and res[1] == 0 and res[2] == 0 and res[3] == 0):
            command = 'CMD_MOTOR#1500#1500#-1300#-1300\n'
            s.send(command.encode('utf-8'))
            time.sleep(0.8)
            command = 'CMD_MOTOR#-1000#-1000#-1000#-1000\n'
            s.send(command.encode('utf-8'))
            time.sleep(0.5)
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))
            time.sleep(0.1)
        elif (abs(res[0]-80)>20 and res[2]<30 and res[3]<30):
            v = 0
            w = -35 * (80 - res[0])
            u = np.array([v - w, v + w])
            u[u > 1500.] = 1500.
            u[u < -1500.] = -1500.
            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
            s.send(command.encode('utf-8'))
            print(command)
            time.sleep(0.1)
        else:
            v = -2000 * (1-(res[2]*res[3])/(120*160))
            w = -40 * (80 - res[0])
            u = np.array([v - w, v + w])
            u[u > 1500.] = 1500.
            u[u < -1500.] = -1500.
            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
            s.send(command.encode('utf-8'))
            print(command)
            time.sleep(0.1)



if __name__ == "__main__":
    clientAddress = "192.168.0.21"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 320

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

    interface = rpc.rpc_network_master(slave_ip=omv_ip, my_ip="", port=0x1DBA)

    start = (5, 0)
    end = (0, 7)

    # P controller
    K1 = -1100
    K2 = -50

    try:
        not_there = True
        cur_node = 1
        while not_there:
            if robot_id in positions:
                # current position
                xy = np.array(positions[robot_id])
                # current rotation
                theta = rotations[robot_id] * np.pi/180
                print('Current Node', cur_node)
                print('Robot position', xy[0], xy[1])
                # print('Robot rotation', theta)

                # Calculate the angle to the next waypoint in the route
                desired_x = route[cur_node][0]
                desired_y = route[cur_node][1]
                # desired_x = 5.3
                # desired_y = 0
                print("Desired position", desired_x, desired_y)

                error_x = desired_x - xy[0]
                error_y = desired_y - xy[1]
                print('Error', error_x, error_y)

                alpha = np.arctan2(error_y, error_x)
                # angle to destination in robot frame
                diff = alpha - theta
                # print('Alpha', alpha)
                error_w = np.degrees(np.arctan2(np.sin(diff), np.cos(diff)))
                # print('Error w', error_w)
                
                v = K1 * (np.sqrt(error_x**2 + error_y**2))
                w = K2 * error_w
                u = np.array([v - w, v + w])
                u[u > 1500.] = 1500.
                u[u < -1500.] = -1500.
                
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                print(command)
                time.sleep(0.1)

                if abs(error_x)<0.4 and abs(error_y)<0.4:
                    print('Hit waypoint', cur_node, route[cur_node])
                    print('Moving to next waypoint', cur_node, route[cur_node])
                    cur_node+=1
                    if cur_node == len(route):
                        not_there = False
                        break

        # start_search = time.time()
        # print("Searching")
        # while(time.time() - start_search < 60):
        #     sys.stdout.flush()
        #     exe_duck_detection(interface)
        #     # pygame.init()
        #     # screen_w = 640
        #     # screen_h = 480
        #     # # screen_w = 160
        #     # # screen_h = 120
        #     # try:
        #     #     screen = pygame.display.set_mode((screen_w, screen_h), flags=pygame.RESIZABLE)
        #     # except TypeError:
        #     #     screen = pygame.display.set_mode((screen_w, screen_h))
        #     # pygame.display.set_caption('Frame Buffer')
        #     # clock = pygame.time.Clock()
        #     # result = interface.call('jpeg_image_stream', 'sensor.RGB565,sensor.QQVGA')
        #     # if result is not None:# THE REMOTE DEVICE WILL START STREAMING ON SUCCESS. SO, WE NEED TO RECEIVE DATA IMMEDIATELY.
        #     #     interface.stream_reader(jpg_frame_buffer_cb, queue_depth=8)
        #     # for event in pygame.event.get():
        #     #     if event.type == pygame.QUIT:
        #     #         pygame.quit()
        #     #         quit()

        
        # not_there = True
        # cur_node = len(route)-2
        # while not_there:
        #     if robot_id in positions:
        #         # current position
        #         xy = np.array(positions[robot_id])
        #         # current rotation
        #         theta = rotations[robot_id] * np.pi/180
        #         print('Current Node', cur_node)
        #         print('Robot position', xy[0], xy[1])
        #         # print('Robot rotation', theta)

        #         # Calculate the angle to the next waypoint in the route
        #         desired_x = route[cur_node][0]
        #         desired_y = route[cur_node][1]
        #         # desired_x = 5.3
        #         # desired_y = 0
        #         print("Desired position", desired_x, desired_y)

        #         error_x = desired_x - xy[0]
        #         error_y = desired_y - xy[1]
        #         print('Error', error_x, error_y)

        #         alpha = np.arctan2(error_y, error_x)
        #         # angle to destination in robot frame
        #         diff = alpha - theta
        #         # print('Alpha', alpha)
        #         error_w = np.degrees(np.arctan2(np.sin(diff), np.cos(diff)))
        #         # print('Error w', error_w)
                
        #         v = (1.5) * K1 * (np.sqrt(error_x**2 + error_y**2))
        #         w = (1.3) * K2 * error_w
        #         u = np.array([v - w, v + w])
        #         u[u > 1500.] = 1500.
        #         u[u < -1500.] = -1500.
                
        #         command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
        #         s.send(command.encode('utf-8'))
        #         print(command)
        #         time.sleep(0.1)

        #         if abs(error_x)<0.4 and abs(error_y)<0.4:
        #             print('Hit waypoint', cur_node, route[cur_node])
        #             print('Moving to next waypoint', cur_node, route[cur_node])
        #             cur_node-=1
        #             if cur_node == -1:
        #                 not_there = False
        #                 break
                    
        # command = 'CMD_MOTOR#1600#1600#1600#1600\n'
        # s.send(command.encode('utf-8'))
        # time.sleep(2)
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