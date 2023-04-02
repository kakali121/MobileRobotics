import sys
from math import *
import numpy as np
import socket
import time
import paho.mqtt.client as mqtt

IP_ADDRESS = '192.168.0.212'
clientAddress = "192.168.0.23"
MQTTHOST = "192.168.0.36"
MQTTPORT = 1883
MQTTTOPIC = "duck"
mqttClient = mqtt.Client()

found = False

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

def Left_Turn():
    command = 'CMD_MOTOR#-1500#-1500#1700#1700\n'
    s.send(command.encode('utf-8'))
    
def Right_Turn():
    command = 'CMD_MOTOR#1500#1500#-1300#-1300\n'
    s.send(command.encode('utf-8'))

def Forward():
    command = 'CMD_MOTOR#2000#2000#2000#2000\n'
    print(command)
    s.send(command.encode('utf-8'))

def Terminate():
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    mqttClient.subscribe(MQTTTOPIC)

def on_message(client, userdata, msg):
    ball = str(msg.payload.decode("utf-8"))
    val = [eval(i) for i in ball.split(' ')]
    print(val)
    Proceed(val[0], val[1])

def Proceed(width, size):
    # if size > 4000 and width > 0:
    #     Left_Turn()
    #     time.sleep(1)
    #     Terminate()
    #     time.sleep(2)
    # else:
    v = 900 *(1+size/13500)
    w = 0.26 * (size-1000)
    u = np.array([v-w, v+w])
    print("u: ", u[0], u[1])
    u[u > 1300.] = 1300.
    u[u < -1300.] = -1300.
    print("u actual: ", u[0], u[1])
    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
    s.send(command.encode('utf-8'))
    # print(command)
    # time.sleep(0.1)
    # Terminate()


def Orien(width, area, center, size):
    if size > 100:
        Proceed(width, area)
    # print("Center", center)
    else:
        w = 25 * center
        u = np.array([1200-w, 1200+w])
        print("u: ", u[0], u[1])
        u[u > 1200.] = 1200.
        u[u < -1200.] = -1200.
        print("u actual: ", u[0], u[1])
        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
        s.send(command.encode('utf-8'))
        # print(command)
        # time.sleep(0.1)
        # Terminate()

start = time.time()
count = 0

if __name__ == '__main__':
    try:
        mqttClient.on_connect = on_connect
        mqttClient.on_message = on_message
        mqttClient.connect("test.mosquitto.org", MQTTPORT)
        mqttClient.loop_start()
        while (time.time() - start < 600):
            temp = int(time.time() - start)
            if temp != count:
                count = temp
                print("Time elaspsed:", count)
            pass
        mqttClient.loop_stop()
    except KeyboardInterrupt:
        Terminate()
        sys.exit(0)

print("Done")
# Close the connection
s.shutdown(2)
s.close()