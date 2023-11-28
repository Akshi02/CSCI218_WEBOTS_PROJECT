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
from sys import maaxint

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
ul_sensor_back = UltrasonicSensor(INPUT_7)
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

    location = gps_sensor.y

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

    while gps_sensor.y < location + 40:
        tank_drive.on(20, 20)

    tank_drive.on(0, 0)


def move_down():
    global facing

    location = gps_sensor.y

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

    while gps_sensor.y < location - 40:
        tank_drive.on(20, 20)

    tank_drive.on(0, 0)


def move_left():
    location = gps_sensor.x

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

    while gps_sensor.x < location - 40:
        tank_drive.on(20, 20)

    tank_drive.on(0, 0)


def move_right():
    location = gps_sensor.x

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

    while gps_sensor.x < location + 40:
        tank_drive.on(20, 20)

    tank_drive.on(0, 0)


def is_valid_action(state, next_state):
    if (
        next_state[0] < -7
        or next_state[0] > 7
        or next_state[1] < -7
        or next_state[1] > 7
    ):
        return False
    elif (state, next_state) in wall_states:
        return False
    else:
        return True


def nextMove(next_state, next_action):
    if next_action == 0:
        move_up()
    elif next_action == 1:
        move_right()
    elif next_action == 2:
        move_down()
    elif next_action == 3:
        move_right()


def get_current_state():
    return get_state(gps_sensor.x, gps_sensor.y)


# Akshita's Functions --------------------------------------------------------------------------------------------------------------------------------------


def get_state(x, y):
    return (x // 40, y // 40)


# next step: to use get_state to remove the wall state from the maze state make a function of it.


def wall_tracker(x, y):
    global facing

    if ul_sensor_front.distance_centimeters < 40:
        if facing == 0:
            wall_states.append(((x, y), (x, y + 1)))
        elif facing == 2:
            wall_states.append(((x, y), (x, y - 1)))
        elif facing == 1:
            wall_states.append(((x, y), (x + 1, y)))
        elif facing == 3:
            wall_states.append(((x, y), (x - 1, y)))

    if ul_sensor_right.distance_centimeters < 40:
        if facing == 0:
            wall_states.append(((x, y), (x + 1, y)))
        elif facing == 2:
            wall_states.append(((x, y), (x - 1, y)))
        elif facing == 1:
            wall_states.append(((x, y), (x, y - 1)))
        elif facing == 3:
            wall_states.append(((x, y), (x, y + 1)))

    if ul_sensor_left.distance_centimeters < 40:
        if facing == 0:
            wall_states.append(((x, y), (x - 1, y)))
        elif facing == 2:
            wall_states.append(((x, y), (x + 1, y)))
        elif facing == 1:
            wall_states.append(((x, y), (x, y + 1)))
        elif facing == 3:
            wall_states.append(((x, y), (x, y - 1)))

    if ul_sensor_back.distance_centimeters < 40:
        if facing == 0:
            wall_states.append(((x, y), (x, y - 1)))
        elif facing == 2:
            wall_states.append(((x, y), (x, y + 1)))
        elif facing == 1:
            wall_states.append(((x, y), (x - 1, y)))
        elif facing == 3:
            wall_states.append(((x, y), (x + 1, y)))


# Akshita's States Defination:

maze_width = 15
maze_height = 15

# Define states
states = [(x - 7, y - 7) for x in range(maze_width) for y in range(maze_height)]
goal_state = (7, 7)
# ------------------------------------------------------------------------------------------------------------------------------
# Joseph's code

q_table = [[0, 0, 0, 0], [0, 0, 0, 0]]

gamma = 0.8

# reward = -100

visited = []


def computeReward(state):
    if state == goal_state:
        return 100

    elif state in visited:
        return -10

    else:
        return 10


def getNextStates(state):  # returns the q values of next possible states
    stateLeft = (state[0] - 1, state[1])
    stateRight = (state[0] + 1, state[1])
    stateUp = (state[0], state[1] + 1)
    stateDown = (state[0], state[1] - 1)

    q0 = computeReward(stateLeft) + gamma * max(
        q_table[15 * stateLeft[0] + stateLeft[1]][0],
        q_table[15 * stateLeft[0] + stateLeft[1]][1],
        q_table[15 * stateLeft[0] + stateLeft[1]][2],
        q_table[15 * stateLeft[0] + stateLeft[1]][3],
    )
    q1 = computeReward(stateUp) + gamma * max(
        q_table[15 * stateUp[0] + stateUp[1]][0],
        q_table[15 * stateUp[0] + stateUp[1]][1],
        q_table[15 * stateUp[0] + stateUp[1]][2],
        q_table[15 * stateUp[0] + stateUp[1]][3],
    )
    q2 = computeReward(stateRight) + gamma * max(
        q_table[15 * stateRight[0] + stateRight[1]][0],
        q_table[15 * stateRight[0] + stateRight[1]][1],
        q_table[15 * stateRight[0] + stateRight[1]][2],
        q_table[15 * stateRight[0] + stateRight[1]][3],
    )
    q3 = computeReward(stateDown) + gamma * max(
        q_table[15 * stateDown[0] + stateDown[1]][0],
        q_table[15 * stateDown[0] + stateDown[1]][1],
        q_table[15 * stateDown[0] + stateDown[1]][2],
        q_table[15 * stateDown[0] + stateDown[1]][3],
    )

    next_states = (q0, q1, q2, q3)

    return next_states


def computeQValue(state, action):
    # q_table = [[0,0,0,0],[0,0,0,0]] list of states and their actions inside them

    # q(state, action) = r(state, action) + gamma * Max[Q_nextState()]

    next_state = state

    if action == 0:  # go up
        next_state[1] = next_state[1] + 1

    elif action == 1:  # go right
        next_state[0] = next_state[0] + 1

    elif action == 2:  # go down
        next_state[1] = next_state[1] - 1

    else:  # go left
        next_state[0] = next_state[0] - 1

    if is_valid_action(state, next_state):
        reward = computeReward(next_state)

        q = reward + gamma * max(getNextStates(next_state))

        q_table[state][action] = q

        return q

    else:
        return _(-maxint - 1)


def get_q_value(state, action):
    return q_table[state][action]


def q_training():
    run = True

    while run:
        # get state

        next_state = 0
        next_action = 0

        current_state = get_current_state()

        # checking if the state is valid

        Q_up = computeQValue(current_state, 0)
        Q_right = computeQValue(current_state, 1)
        Q_down = computeQValue(current_state, 2)
        Q_left = computeQValue(current_state, 3)

        best_Q = Q_up

        Q_options = (Q_right, Q_down, Q_left)

        for i in range(len(Q_options)):
            if Q_options[i] > best_Q:
                next_state = Q_options[i]
                next_action = i

        # need to stop move incase robot reaches goal state (Break Loop)

        nextMove(next_state, next_action)

        if get_current_state() == goal_state:
            run = False


# uses trained q table to solve the maze
def q_testing():
    run = True

    while run:
        next_state = 0
        next_action = 0

        current_state = get_current_state

        Q_up = get_q_value(current_state, 0)
        Q_right = get_q_value(current_state, 1)
        Q_down = get_q_value(current_state, 2)
        Q_left = get_q_value(current_state, 3)

        best_Q = Q_up

        Q_options = (Q_right, Q_down, Q_left)

        for i in range(len(Q_options)):
            if Q_options[i] > best_Q:
                next_state = Q_options[i]
                next_action = i

        nextMove(next_state, next_action)

        if get_current_state() == goal_state:
            run = False


def main():
    q_training()


main()
