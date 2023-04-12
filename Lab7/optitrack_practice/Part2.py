import sys
from math import *
import numpy as np
import socket
import time
import paho.mqtt.client as mqtt
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

IP_ADDRESS = '192.168.0.212'
clientAddress = "192.168.0.23"
MQTTHOST = "192.168.0.36"
MQTTPORT = 1883
MQTTTOPIC = "duck"
mqttClient = mqtt.Client()

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
K1 = 120
K2 = 1500

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    mqttClient.subscribe(MQTTTOPIC)

def on_message(client, userdata, msg):
    duck = str(msg.payload.decode("utf-8"))
    val = [eval(i) for i in duck.split(' ')]
    Map(val[0], val[1])

def Map(duck_the, duck_dist):
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

            v = K1*(sqrt(errorx**2 + errory**2))
            w = K2 * errorw

            u = np.array([v - w, v + w])
            u[u > 1500.] = 1500.
            u[u < -1500.] = -1500.
            
            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
            s.send(command.encode('utf-8'))
            time.sleep(0.1)


def Terminate():
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

if __name__ == '__main__':
    try:
        mqttClient.on_connect = on_connect
        mqttClient.on_message = on_message
        mqttClient.connect("test.mosquitto.org", MQTTPORT)
        mqttClient.loop_start()
        start = time.time()
        count = 0
        while (time.time() - start < 600):
            temp = int(time.time() - start)
            if temp != count:
                count = temp
                print("Time elaspsed:", count)
            pass
        mqttClient.loop_stop()
        Terminate()
    except KeyboardInterrupt:
        Terminate()
        sys.exit(0)
        streaming_client.shutdown()

print("Done")
# Close the connection
s.shutdown(2)
s.close()