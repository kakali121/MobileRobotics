import Motor
import time
import Ultrasonic

PWM = Motor.Motor()
sensor = Ultrasonic.Ultrasonic()

def destroy():
    PWM.setMotorModel(0, 0, 0, 0)
    
def prog1():
    #Slow
    PWM.setMotorModel(750, 750, 750, 750)
    time.sleep(2)
    destroy()
    PWM.setMotorModel(-750, -750, -750, -750)
    time.sleep(2)
    destroy()
    #Fast
    PWM.setMotorModel(1000, 1000, 1000, 1000)
    time.sleep(2)
    destroy()
    PWM.setMotorModel(-1000, -1000, -1000, -1000)
    time.sleep(2)
    destroy()
    #Aggressive
    PWM.setMotorModel(1500, 1500, 1500, 1500)
    time.sleep(2)
    destroy()
    PWM.setMotorModel(-1500, -1500, -1500, -1500)
    time.sleep(2)
    destroy()
    
def prog2():
    while True:
        print(sensor.get_distance())
        time.sleep(2)
    
def prog3():
    distance = sensor.get_distance()
    start = time.time()
    while distance != 50:
        u = 200 * (distance - 50) 
        print(distance, " cm")
        PWM.setMotorModel(u, u, u, u)    
        distance = sensor.get_distance()
    end = time.time()
    print(end - start)
    PWM.setMotorModel(0,0,0,0)
    
if __name__== '__main__':
    try:
        #prog1()
        #prog2()
        prog3()
        #k = 100, 150, 200
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()

