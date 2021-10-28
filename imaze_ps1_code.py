"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import Camera
import numpy as np
import cv2
import math
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
left_motor=robot.getDevice('left wheel motor')
right_motor=robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

max_speed=6.27

prox_sensor=[]
for i in range(8):
    name='ps'+str(i)
    prox_sensor.append(robot.getDevice(name))
    prox_sensor[i].enable(timestep)

    
def moveForward(speed):
    if(speed>100):
         speed=100
    speed=(max_speed/100.0)*speed
    for _ in range(10):
          left_motor.setVelocity(speed)
          right_motor.setVelocity(speed)
          
def moveRight(speed):
    if(speed>100):
         speed=100
    speed=(max_speed/100.0)*speed
    for _ in range(10):
          left_motor.setVelocity(speed)
          right_motor.setVelocity(-speed)
          
def moveSlightRight(speed):
    if(speed>100):
         speed=100
    speed=(max_speed/100.0)*speed
    for _ in range(10):
          left_motor.setVelocity(speed)
          right_motor.setVelocity(speed/1.5)

def moveSlightLeft(speed):
    if(speed>100):
         speed=100
    speed=(max_speed/100.0)*speed
    for _ in range(10):
          left_motor.setVelocity(speed/1.5)
          right_motor.setVelocity(speed)          
# Main loop:

def frontIrReading():
    return math.floor(prox_sensor[0].getValue()+prox_sensor[7].getValue())

def leftIrReading():
    return math.floor(prox_sensor[5].getValue())
    
def rightIrReading():
    return math.floor(prox_sensor[2].getValue())
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    right_wall=rightIrReading()>90
    front_wall=frontIrReading()>150
    front_wallfar=frontIrReading()>140
    left_wall=leftIrReading()>100
    
    if front_wall:
       moveRight(100)
       
    elif left_wall :
       moveSlightRight(100)
    elif right_wall:
       moveSlightLeft(100)
    else :
       moveForward(100)
       
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # rgb=camera.getImageArray()
    
    # Process sensor data here.
    # moveForward(100)
    
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
