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

from sys import maxint

from math import pi

def circle_angle(gyro):
        """
        As the gryo rotates clockwise the angle increases, it will increase
        by 360 for each full rotation. As the gyro rotates counter-clockwise
        the gyro angle will decrease.

        The angles on a circle have the opposite behavior though, they start
        at 0 and increase as you move counter-clockwise around the circle.

        Convert the gyro angle to the angle on a circle. We consider the initial
        position of the gyro to be at 90 degrees on the cirlce.
        """
        current_angle = gyro.angle
        delta = abs(current_angle - init_angle) % 360

        if delta == 0:
            result = 90

        # the gyro has turned clockwise relative to where we started
        elif current_angle > init_angle:

            if delta <= 90:
                result = 90 - delta

            elif delta <= 180:
                result = 360 - (delta - 90)

            elif delta <= 270:
                result = 270 - (delta - 180)

            else:
                result = 180 - (delta - 270)

            # This can be chatty (but helpful) so save it for a rainy day
            # log.info("%s moved clockwise %s degrees to %s" % (self, delta, result))

        # the gyro has turned counter-clockwise relative to where we started
        else:
            if delta <= 90:
                result = 90 + delta

            elif delta <= 180:
                result = 180 + (delta - 90)

            elif delta <= 270:
                result = 270 + (delta - 180)

            else:
                result = delta - 270

            # This can be chatty (but helpful) so save it for a rainy day
            # log.info("%s moved counter-clockwise %s degrees to %s" % (self, delta, result))

        return result

class Wheel(object):
    """
    A base class for various types of wheels, tires, etc.  All units are in mm.

    One scenario where one of the child classes below would be used is when the
    user needs their robot to drive at a specific speed or drive for a specific
    distance. Both of those calculations require the circumference of the wheel
    of the robot.

    Example:

    .. code:: python

        from ev3dev2.wheel import EV3Tire

        tire = EV3Tire()

        # calculate the number of rotations needed to travel forward 500 mm
        rotations_for_500mm = 500 / tire.circumference_mm
    """
    def __init__(self, diameter_mm, width_mm):
        self.diameter_mm = float(diameter_mm)
        self.width_mm = float(width_mm)
        self.circumference_mm = diameter_mm * pi

    @property
    def radius_mm(self):
        return float(self.diameter_mm / 2)

class EV3Tire(Wheel):
    """
    part number 44309
    comes in set 31313
    """
    def __init__(self):
        Wheel.__init__(self, 43.2, 21)

class MoveDifferential(MoveTank):
    def __init__(self,
                 left_motor_port,
                 right_motor_port,
                 wheel_class,
                 wheel_distance_mm,
                 gyro=None,
                 desc=None,
                 motor_class=LargeMotor):

        MoveTank.__init__(self, left_motor_port, right_motor_port, desc, motor_class)
        self.wheel = wheel_class()
        self.wheel_distance_mm = wheel_distance_mm

        # The circumference of the circle made if this robot were to rotate in place
        self.circumference_mm = self.wheel_distance_mm * math.pi

        self.min_circle_radius_mm = self.wheel_distance_mm / 2

        # odometry variables
        self.x_pos_mm = 0.0  # robot X position in mm
        self.y_pos_mm = 0.0  # robot Y position in mm
        self.odometry_thread_run = False
        self.theta = 0.0
        self._gyro = gyro
    
    def on_for_distance(self, speed, distance_mm, brake=True, block=True):
        """
        Drive in a straight line for ``distance_mm``
        """
        rotations = distance_mm / self.wheel.circumference_mm
        
        MoveTank.on_for_rotations(self, speed, speed, rotations, brake, block)

    def turn_degrees(self, speed, degrees, brake=True, block=True, error_margin=2, use_gyro=False):
        """
        Rotate in place ``degrees``. Both wheels must turn at the same speed for us
        to rotate in place.  If the following conditions are met the GryoSensor will
        be used to improve the accuracy of our turn:
        - ``use_gyro``, ``brake`` and ``block`` are all True
        - A GyroSensor has been defined via ``self.gyro = GyroSensor()``
        """
        def final_angle(init_angle, degrees):
            result = init_angle - degrees

            while result <= -360:
                result += 360

            while result >= 360:
                result -= 360

            if result < 0:
                result += 360

            return result

        # use the gyro to check that we turned the correct amount?
        use_gyro = bool(use_gyro and block and brake)
        if use_gyro and not self._gyro:
            raise DeviceNotDefined(
                "The 'gyro' variable must be defined with a GyroSensor. Example: tank.gyro = GyroSensor()")

        if use_gyro:
            angle_init_degrees = circle_angle(self._gyro)
        else:
            angle_init_degrees = math.degrees(self.theta)

        angle_target_degrees = final_angle(angle_init_degrees, degrees)

        # The distance each wheel needs to travel
        distance_mm = (abs(degrees) / 360) * self.circumference_mm

        # The number of rotations to move distance_mm
        rotations = distance_mm / self.wheel.circumference_mm

        # If degrees is positive rotate clockwise
        if degrees > 0:
            MoveTank.on_for_rotations(self, speed, speed * -1, rotations, brake, block)

        # If degrees is negative rotate counter-clockwise
        else:
            MoveTank.on_for_rotations(self, speed * -1, speed, rotations, brake, block)

        if use_gyro:
            angle_current_degrees = circle_angle(self._gyro)

            # This can happen if we are aiming for 2 degrees and overrotate to 358 degrees
            # We need to rotate counter-clockwise
            if 90 >= angle_target_degrees >= 0 and 270 <= angle_current_degrees <= 360:
                degrees_error = (angle_target_degrees + (360 - angle_current_degrees)) * -1

            # This can happen if we are aiming for 358 degrees and overrotate to 2 degrees
            # We need to rotate clockwise
            elif 360 >= angle_target_degrees >= 270 and 0 <= angle_current_degrees <= 90:
                degrees_error = angle_current_degrees + (360 - angle_target_degrees)

            # We need to rotate clockwise
            elif angle_current_degrees > angle_target_degrees:
                degrees_error = angle_current_degrees - angle_target_degrees

            # We need to rotate counter-clockwise
            else:
                degrees_error = (angle_target_degrees - angle_current_degrees) * -1

            if abs(degrees_error) > error_margin:
                self.turn_degrees(speed, degrees_error, brake, block, error_margin, use_gyro)

STUD_MM = 8

gyro_sensor = GyroSensor(INPUT_5)
init_angle = gyro_sensor.angle

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 16 * STUD_MM, gyro = gyro_sensor)
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


motorC = LargeMotor(OUTPUT_C)  # Magnet

# Here is where your code starts
ul_sensor_front.MODE_US_DIST_CM = "US-DIST-CM"
ul_sensor_left.MODE_US_DIST_CM = "US-DIST-CM"
ul_sensor_right.MODE_US_DIST_CM = "US-DIST-CM"

# facing represents which direction the robot is facing
# 0 represents up
# 1 represents right
# 2 represents down
# 3 represents left
facing = 0

wall_states = set()

cell_size = 60

move_speed = 30

# Not to be used. These functions are for making the actions simpler to read.
def turn_right_90():
    global facing


    #angle = gyro_sensor.angle
    #while gyro_sensor.angle < angle + 87:
        #steering_drive.on(100, 20)
    #tank_drive.on(0, 0)

    tank_drive.turn_degrees(30, 90, use_gyro=True) 
    
    facing += 1
    facing %= 4


def turn_left_90():
    global facing
    
    #angle = gyro_sensor.angle
    #while gyro_sensor.angle > angle - 85:
        #tank_drive.on(-10, 20)
    #tank_drive.on(0, 0)
    
    tank_drive.turn_degrees(30, -90, use_gyro=True)
    
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

    while gps_sensor.y < location + cell_size:
        tank_drive.on(move_speed, move_speed)

    tank_drive.on(0, 0)

def move_down():
    global facing

    location = gps_sensor.y

    # facing up
    if facing == 0:
        turn_right_90()
        turn_right_90()
    # facing right
    elif facing == 1:
        turn_left_90()
    # facing left
    elif facing == 3:
        turn_left_90()

    while gps_sensor.y > location - cell_size:
        tank_drive.on(move_speed, move_speed)

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
        turn_right_90()

    while gps_sensor.x > location - cell_size:
        tank_drive.on(move_speed, move_speed)

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

    while gps_sensor.x < location + cell_size:
        tank_drive.on(move_speed, move_speed)

    tank_drive.on(0, 0)


def is_valid_action(state, next_state):
    if (
        next_state[0] < 0
        or next_state[0] > 14
        or next_state[1] < 0
        or next_state[1] > 14
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
        move_left()


def get_current_state():
    return get_state(gps_sensor.x, gps_sensor.y)


# Akshita's Functions --------------------------------------------------------------------------------------------------------------------------------------


def get_state(x, y):
    return (int(x // cell_size)+7, int(y // cell_size)+7)


# next step: to use get_state to remove the wall state from the maze state make a function of it.


def wall_tracker(x, y):
    global facing

    if ul_sensor_front.distance_centimeters < cell_size:
        if facing == 0:
            wall_states.add(((x, y), (x, y + 1)))
        elif facing == 2:
            wall_states.add(((x, y), (x, y - 1)))
        elif facing == 1:
            wall_states.add(((x, y), (x + 1, y)))
        elif facing == 3:
            wall_states.add(((x, y), (x - 1, y)))

    if ul_sensor_right.distance_centimeters < cell_size:
        if facing == 0:
            wall_states.add(((x, y), (x + 1, y)))
        elif facing == 2:
            wall_states.add(((x, y), (x - 1, y)))
        elif facing == 1:
            wall_states.add(((x, y), (x, y - 1)))
        elif facing == 3:
            wall_states.add(((x, y), (x, y + 1)))

    if ul_sensor_left.distance_centimeters < cell_size:
        if facing == 0:
            wall_states.add(((x, y), (x - 1, y)))
        elif facing == 2:
            wall_states.add(((x, y), (x + 1, y)))
        elif facing == 1:
            wall_states.add(((x, y), (x, y + 1)))
        elif facing == 3:
            wall_states.add(((x, y), (x, y - 1)))

    if ul_sensor_back.distance_centimeters < cell_size:
        if facing == 0:
            wall_states.add(((x, y), (x, y - 1)))
        elif facing == 2:
            wall_states.add(((x, y), (x, y + 1)))
        elif facing == 1:
            wall_states.add(((x, y), (x - 1, y)))
        elif facing == 3:
            wall_states.add(((x, y), (x + 1, y)))


# Akshita's States Defination:

maze_width = 15
maze_height = 15

# Define states
states = [(x, y) for x in range(maze_width) for y in range(maze_height)]
goal_state = (14, 14)
# ------------------------------------------------------------------------------------------------------------------------------
# Joseph's code

q_table = [[0, 0, 0, 0] for i in range(225)]

gamma = 0.8

# reward = -100

visited = []

def computeReward(state):
    if state == goal_state:
        return 100

    elif state in visited:
        return -10

    else:
        visited.append(state)
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

    next_state = get_next_state(state, action)

    if is_valid_action(state, next_state):
        reward = computeReward(next_state)

        q = reward + gamma * max(getNextStates(next_state))

        q_table[15 * state[0] + state[1]][action] = q

        return q

    else:
        return (-maxint - 1)


def get_q_value(state, action):
    return q_table[15 * state[0] + state[1]][action]

def get_next_state(state, action):
    if action == 0:  # go up
        return(state[0], state[1] + 1)

    elif action == 1:  # go right
        return(state[0] + 1, state[1])

    elif action == 2:  # go down
        return(state[0], state[1] - 1)

    else:  # go left
        return(state[0] - 1, state[1])

def q_training():
    run = True

    while run:
        # get state

        next_state = 0
        next_action = 0

        current_state = get_current_state()

        #get surrounding walls
        wall_tracker(x, y)
        
        # checking if the state is valid

        Q_up = computeQValue(current_state, 0)
        Q_right = computeQValue(current_state, 1)
        Q_down = computeQValue(current_state, 2)
        Q_left = computeQValue(current_state, 3)

        best_Q = Q_up

        Q_options = (Q_right, Q_down, Q_left)

        for i in range(len(Q_options)):
            if Q_options[i] > best_Q:
                next_state = get_next_state(current_state, i)
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

        current_state = get_current_state()

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
    #q_training()
    
    for i in range(10):
        next_state = 0
        next_action = 0

        current_state = get_current_state()
    
        #get surrounding walls
        wall_tracker(current_state[0], current_state[1])

        Q_up = computeQValue(current_state, 0)
        Q_right = computeQValue(current_state, 1)
        Q_down = computeQValue(current_state, 2)
        Q_left = computeQValue(current_state, 3)
    
        best_Q = Q_up

        Q_options = (Q_right, Q_down, Q_left)
    
        print(best_Q)
        print(Q_options)

        for i in range(len(Q_options)):
            if Q_options[i] > best_Q:
                next_state = get_next_state(current_state, i+1)
                next_action = i+1

        print()
        print(next_state)
        print(next_action)
    
        nextMove(next_state, next_action)

        if get_current_state() == goal_state:
            run = False

main()

