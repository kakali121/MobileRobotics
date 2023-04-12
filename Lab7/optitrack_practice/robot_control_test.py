import socket
import time

IP_ADDRESS = '192.168.0.212'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')


try:
    while True:
        # Send control input to the motors
        command = 'CMD_MOTOR#1500#1500#1500#1500\n'
        s.send(command.encode('utf-8'))

        # Wait for 1 second
        time.sleep(1)


except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

# Close the connection
s.shutdown(2)
s.close()
