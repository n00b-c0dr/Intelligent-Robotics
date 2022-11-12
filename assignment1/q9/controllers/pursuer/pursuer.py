from controller import Robot


import q1

from sympy.geometry import Ray
import numpy as np

robot = Robot()
timestep = 32
max_speed = 6.28


def can_move_forward(is_obstacle):
    for i in range(len(is_obstacle)):
        if i == 3 or i == 4:
            continue
        if is_obstacle[i]:
            return False
    return True


def check_obstacle(prox_sensors):
    readings = []
    for i in range(len(prox_sensors)):
        if prox_sensors[i].getValue() >= 80:
            readings.append(True)
        else:
            readings.append(False)
    return readings

def init_robot(robot, timestep):
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    motors = {'left': left_motor, 'right': right_motor}

    gps = robot.getDevice('gps')
    gps.enable(timestep)

    inertial = robot.getDevice("inertial unit")
    inertial.enable(timestep)

    prox_sensors = []
    for i in range(8):
        sensor_name = 'ps' + str(i)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[i].enable(timestep)

    return motors, gps, inertial, prox_sensors


motors, gps, inertial, prox_sensors = init_robot(robot, timestep)

def get_turn_angle(current_location, current_orientation, desired_location):
    forward_ray = Ray(current_location, angle=current_orientation)
    desired_ray = Ray(current_location, desired_location)
    angle = float(forward_ray.angle_between(desired_ray))
    if float(desired_ray.angle_between(Ray(current_location, angle=angle))) == 0.0:
        pass
    else:
        angle = -angle
    return angle



cameras = [(-0.4, 0.8), (0.4, 0.8), (0.4, -0.3), 
           (0.85, -0.3), (1.1, 0.85), (1.6, 0.85), (1.5, -0.85), (-0.5, -0.7)]
mode = 0
m_line = None
side_detected = False
choice = 0
goal_location = cameras[0]
print(goal_location)
while robot.step(timestep) != -1:
    current_location = [round(gps.getValues()[i], 3) for i in range(2)]
    current_orientation = inertial.getRollPitchYaw()[2]
    # is_obstacle = check_obstacle(prox_sensors)
    # while evader not seen
    
    if mode == 0:
            turn_angle = get_turn_angle(current_location, current_orientation, goal_location)
            if abs(turn_angle) > 0.1:
                motors['left'].setVelocity(max_speed)
                motors['right'].setVelocity(-max_speed)
            elif abs(turn_angle) <= 0.1:
                print("Moving Forward")
                mode = 1
    elif mode == 1:
        motors['left'].setVelocity(max_speed)
        motors['right'].setVelocity(max_speed)
        if q1.get_distance_between_points(current_location, goal_location) <= 0.15:
            print("Reached Goal!!!")
            motors['left'].setVelocity(0)
            motors['right'].setVelocity(0)
            choice = (choice + 1) % 8
            goal_location = cameras[choice]
            mode = 0
    else:
        motors['left'].setVelocity(max_speed)
        motors['right'].setVelocity(max_speed)

    