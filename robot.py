#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
radio = Radio()

#color_sensor_in1 = ColorSensor(INPUT_1)
#color_sensor_in2 = ColorSensor(INPUT_2)
ul_sensor_front = UltrasonicSensor(INPUT_2)
ul_sensor_left = UltrasonicSensor(INPUT_3)
ul_sensor_right = UltrasonicSensor(INPUT_4)
gps_sensor = GPSSensor(INPUT_6)
gyro_sensor = GyroSensor(INPUT_5)

motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where your code starts
ul_sensor_front.MODE_US_DIST_CM = 'US-DIST-CM'
ul_sensor_left.MODE_US_DIST_CM = 'US-DIST-CM'
ul_sensor_right.MODE_US_DIST_CM = 'US-DIST-CM'

#face_angle = gyro_sensor.angle

#facing represents which direction the robot is facing
# 0 represents up 
# 1 represents right 
# 2 represents down 
# 3 represents left
facing = 0

#Not to be used. These functions are for making the actions simpler to read.
def turn_right_90(): 
    global facing
    
    angle = gyro_sensor.angle
    while gyro_sensor.angle < angle + 90:
        tank_drive.on(40, 0)
    tank_drive.on(0,0)
    facing += 1
    facing %= 4
    
def turn_left_90(): 
    global facing
    
    angle = gyro_sensor.angle
    while gyro_sensor.angle > angle - 90:
        tank_drive.on(0, 40)
    tank_drive.on(0,0)
    facing -= 1
    facing %= 4
    
#Actions
def move_up(): 
    global facing
    
    #facing right
    if facing == 1: 
        turn_left_90()
    #facing down
    elif facing == 2: 
        turn_left_90()
        turn_left_90()
    #facing left
    elif facing == 3: 
        turn_right_90()
        
    tank_drive.on(20,20)
    
def move_down(): 
    global facing
    
    #facing up
    if facing == 0: 
        turn_left_90()
        turn_left_90()
    #facing right
    elif facing == 1: 
        turn_right_90()
    #facing left
    elif facing == 3: 
        turn_left_90()
        
    tank_drive.on(20,20)
    
def move_left(): 
    #facing up
    if facing == 0: 
        turn_left_90()
    #facing right
    elif facing == 1: 
        turn_right_90()
        turn_right_90()
    #facing down
    elif facing == 2: 
        turn_left_90()
    
    tank_drive.on(20,20)
    
def move_right(): 
    #facing up
    if facing == 0: 
        turn_right_90()
    #facing down
    elif facing == 2: 
        turn_left_90()
    #facing left
    elif facing == 3: 
        turn_left_90()
        turn_left_90()
    tank_drive.on(20,20)
    
def facevalue():
    global facing

    face_dir = ()

    # facing up & down
    if facing == 0 or facing == 2: 
        face_dir = (0, 1, 0)
    # facing right & left
    elif facing == 1 or facing == 3: 
        face_dir = (1, 0, 0)

    return face_dir

#while True:
#    if color_sensor_in1.reflected_light_intensity >= 40 and color_sensor_in2.reflected_light_intensity == 0:
'''
    if ul_sensor_front.distance_centimeters < 10 and ul_sensor_left.distance_centimeters < ul_sensor_right.distance_centimeters:
        while gyro_sensor.angle < cur_angle + 90:
            tank_drive.on((10), -15)
        cur_angle = gyro_sensor.angle
        #break
        pass
    if ul_sensor_front.distance_centimeters < 10 and ul_sensor_left.distance_centimeters > ul_sensor_right.distance_centimeters:
        while gyro_sensor.angle > cur_angle - 90:
            tank_drive.on((-15), 10)
        cur_angle = gyro_sensor.angle
        #break
        pass
    tank_drive.on((50), 50)
    print(gps_sensor.x)
    print(gps_sensor.y)
    #print("Im in the loop")
    '''
    
#Akshita's States Defination:

maze_width = 15
maze_height = 15

# Define states
states = [(x, y) for x in range(maze_width) for y in range(maze_height)]

# Choose a state as the starting point
start_index = 0  # Change this index as needed
start = states[start_index]

# Distance to target
distance = ul_sensor_front.distance_centimeters

# Direction to target
direction = facevalue()

# Calculate target coordinates
magnitude = math.sqrt(sum(d**2 for d in direction))
target = [round(start[i] + distance * direction[i] / magnitude) for i in range(2)]

# Convert the target coordinates to integers
target = tuple(map(int, target))

print(f"Starting point: {start}")
print(f"Distance to target: {distance}")
print(f"Direction to target: {direction}")
print(f"Target coordinates: {target}")
print(f"GPS Sensor Coordinates: ({gps_sensor.x}, {gps_sensor.y})")

def get_state(x, y):
    return (x//40, y//40)

#next step: to use get_state to remove the wall state from the maze state make a function of it.
