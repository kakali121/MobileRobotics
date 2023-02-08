import time
from PCA9685 import PCA9685
from Led import *
from Motor import *
from Buzzer import *
from Ultrasonic import *

def forward():
    #Move forward for 2 seconds (1).
    PWM.setMotorModel(2000,2000,2000,2000)       #Forward
    time.sleep(2)                                #Wait 2 seconds
    destroy()
    
def rotate():
    #Rotate approximately 90 degrees.
    PWM.setMotorModel(-1500,-1500,2000,2000)     #Left 
    time.sleep(0.8)                              #Wait 0.9 second
    destroy()
     
def light(index, b, g, r):
    led.ledIndex(index, b, g, r)
    
def buzz():
    buzzer=Buzzer()
    buzzer.run('1')
    time.sleep(1)
    buzzer.run('0')

def loop(): 

    # Move forward for 2 seconds (1).
    forward() 
    # Rotate approximately 90 degrees.
    rotate()
    # Display the color red in LED 0.
    light(0x01, 0, 255, 0)

    # Move forward for 2 seconds (2).
    forward()
    # Rotate approximately 90 degrees.
    rotate()
    # Display the color green in LED 1.
    light(0x02, 0, 0, 255)

    # Move forward for 2 seconds (4).
    forward()
    # Rotate approximately 90 degrees.
    rotate()
    # Display the color yellow in LED 2.
    light(0x04, 255, 0, 0)

    # Move forward for 2 seconds (5).
    forward()
    # Rotate approximately 90 degrees.
    rotate()
    # Display the color yellow in LED 3.
    light(0x08, 255, 255, 0)

    destroy()

    # Beep the buzzer for 1 second.
    buzz()

    
def destroy():
    PWM.setMotorModel(0,0,0,0)     

if __name__== '__main__':
    try:
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()

#Measure (using a rule or measure tape) how far your robot is from the starting location.
