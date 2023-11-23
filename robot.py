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

# color_sensor_in1 = ColorSensor(INPUT_1)
# color_sensor_in2 = ColorSensor(INPUT_2)
ul_sensor_front = UltrasonicSensor(INPUT_2)
ul_sensor_left = UltrasonicSensor(INPUT_3)
ul_sensor_right = UltrasonicSensor(INPUT_4)
gps_sensor = GPSSensor(INPUT_6)
gyro_sensor = GyroSensor(INPUT_5)

motorC = LargeMotor(OUTPUT_C)  # Magnet

# Here is where your code starts
ul_sensor_front.MODE_US_DIST_CM = "US-DIST-CM"
ul_sensor_left.MODE_US_DIST_CM = "US-DIST-CM"
ul_sensor_right.MODE_US_DIST_CM = "US-DIST-CM"

# face_angle = gyro_sensor.angle

# facing represents which direction the robot is facing
# 0 represents up
# 1 represents right
# 2 represents down
# 3 represents left
facing = 0

wall_states = []

# Not to be used. These functions are for making the actions simpler to read.
def turn_right_90():
    global facing

    angle = gyro_sensor.angle
    while gyro_sensor.angle < angle + 90:
        tank_drive.on(40, 0)
    tank_drive.on(0, 0)
    facing += 1
    facing %= 4


def turn_left_90():
    global facing

    angle = gyro_sensor.angle
    while gyro_sensor.angle > angle - 90:
        tank_drive.on(0, 40)
    tank_drive.on(0, 0)
    facing -= 1
    facing %= 4


# Actions
def move_up():
    global facing

    # facing right
    if facing == 1:
        turn_left_90()
    # facing down
    elif facing == 2:
        turn_left_90()
        turn_left_90()
    # facing left
    elif facing == 3:
        turn_right_90()

    tank_drive.on(20, 20)


def move_down():
    global facing

    # facing up
    if facing == 0:
        turn_left_90()
        turn_left_90()
    # facing right
    elif facing == 1:
        turn_right_90()
    # facing left
    elif facing == 3:
        turn_left_90()

    tank_drive.on(20, 20)


def move_left():
    # facing up
    if facing == 0:
        turn_left_90()
    # facing right
    elif facing == 1:
        turn_right_90()
        turn_right_90()
    # facing down
    elif facing == 2:
        turn_left_90()

    tank_drive.on(20, 20)


def move_right():
    # facing up
    if facing == 0:
        turn_right_90()
    # facing down
    elif facing == 2:
        turn_left_90()
    # facing left
    elif facing == 3:
        turn_left_90()
        turn_left_90()
    tank_drive.on(20, 20)

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

def get_state(x, y):
    return (x//40, y//40)

#next step: to use get_state to remove the wall state from the maze state make a function of it.

def wall_tracker(x,y):
    global facing
    
    if ul_sensor_front.distance_centimeters < 40:
        if facing == 0:
            y += 1
        elif facing == 2:
            y -= 1
        elif facing == 1:
            x += 1
        elif facing == 3:
            x -= 1
            
    if ul_sensor_right.distance_centimeters < 40:
        if facing == 0:
            x += 1
        elif facing == 2:
            x -= 1
        elif facing == 1:
            y -= 1
        elif facing == 3:
            y += 1
    
    if ul_sensor_left.distance_centimeters < 40:
        if facing == 0:
            x -= 1
        elif facing == 2:
            x += 1
        elif facing == 1:
            y += 1
        elif facing == 3:
            y -= 1
    
    return(wall_states.append((x,y)))

# while True:
#    if color_sensor_in1.reflected_light_intensity >= 40 and color_sensor_in2.reflected_light_intensity == 0:
"""
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
    """

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

'''
wall_coord = wall_tracker(gps_sensor.x, gps_sensor.y)
print(f"Wall coordinates: {wall_coord}")


Wcoord = get_state(wall_coord[0], wall_coord[1])
print(f"Wall states: {Wcoord}")
'''
wall_coord = get_state(gps_sensor.x, gps_sensor.y)

Wcoord = wall_tracker(wall_coord[0], wall_coord[1])
#print(f"Wall state coordinates: {Wcoord}")

#wall_states = []
print(f"Wall states: {wall_states}")

# ------------------------------------------------------------------------------------------------------------------------------
# Joseph's code

q_table = [[0, 0, 0, 0], [0, 0, 0, 0]]

gamma = 0.8

#reward = -100

visited = []


def computeReward(state):
    
    if state == goal_state:
        return 100
    
    elif (state in visited):
        return -10

    else:
        return 10    
    
def getNextStates(state): #returns the q values of next possible states
    
    q0 = computeReward(stateLeft) + gamma * max(q_table[stateleft][0], q_table[stateleft][1], q_table[stateleft][2], q_table[stateleft][3])
    q1 = computeReward(stateUp) + gamma * max(q_table[stateUp][0], q_table[stateUp][1], q_table[stateUp][2], q_table[stateUp][3])
    q2 = computeReward(stateRight) + gamma * max(q_table[stateRight][0], q_table[stateRight][1], q_table[stateRight][2], q_table[stateRight][3])
    q3 = computeReward(stateDown) + gamma * max(q_table[stateDown][0], q_table[stateDown][1], q_table[stateDown][2], q_table[stateDown][3])
    
    next_states = (q0, q1, q2, q3)
    
    return next_states


def q_learning(state, action):
    # q_table = [[0,0,0,0],[0,0,0,0]] list of states and their actions inside them

    # q(state, action) = r(state, action) + gamma * Max[Q_nextState()]

    reward = computeReward(state)
    
    q = reward + gamma * max(getNextStates(state))
    
    q_table[state][action] = q
    
    
def q_training():
    
    
