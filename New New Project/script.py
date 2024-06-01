from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import random
import time
import math
import sys


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def turn_left():
    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]
    target_angle = normalize_angle(initial_angle + math.pi / 2)
    angle = False
    while not angle:
        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]
        current_angle = normalize_angle(current_angle)
        angle = abs(normalize_angle(current_angle - target_angle)) <= 0.01

        sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
        sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
        print("Turning left...")
        time.sleep(0.05)


def turn_right():
    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]
    target_angle = normalize_angle(initial_angle - math.pi / 2)
    angle = False
    while not angle:
        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]
        current_angle = normalize_angle(current_angle)
        angle = abs(normalize_angle(current_angle - target_angle)) <= 0.01

        sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
        sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
        print("Turning right...")
        time.sleep(0.05)


def go_straight():
    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
    sim.setJointTargetVelocity(right_wheel_handle, base_speed)
    # print("Continuing straight")


def stop():
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)
    # print("Stopping")


def turn_randomly(left, right, front):
    print("Intersection detected, choosing direction...")

    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
    sim.setJointTargetVelocity(right_wheel_handle, base_speed)
    time.sleep(1.5)

    options = []
    if front:
        options.append("Front")
    if left:
        options.append("Left")
    if right:
        options.append("Right")

    if options:  # Se ci sono opzioni disponibili
        direction = random.choice(options)

        if direction == "Right":
            turn_right()
        elif direction == "Left":
            turn_left()
        elif direction == "Front":
            go_straight()
    else:
        print("Unexpected Error, Stopping.")
        stop()


def is_free(sensor):
    _, dist, _, _, _ = sim.readProximitySensor(sensor)
    return dist == 0 or dist > MIN_DISTANCE


if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject("sim")

    robot_handle = sim.getObjectHandle("/PioneerP3DX")
    left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
    right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

    left = sim.getObjectHandle("/Left_Proximity_sensor")
    front = sim.getObjectHandle("/Front_Proximity_sensor")
    right = sim.getObjectHandle("/Right_Proximity_sensor")

    base_speed = 1.0
    turn_speed = 0.3
    MIN_DISTANCE = 0.4

    sim.startSimulation()
    time.sleep(0.2)

    try:
        while True:

            front_free = is_free(front)
            left_free = is_free(left)
            right_free = is_free(right)

            if front_free:
                if not left_free and not right_free:
                    go_straight()
                else:
                    turn_randomly(left=left_free,
                                  front=front_free, right=right_free)
            else:
                if not left_free and not right_free:
                    stop()
                elif left_free and not right_free:
                    turn_left()
                elif right_free and not left_free:
                    turn_right()
                elif left_free and right_free:
                    turn_randomly(left=left_free,
                                  front=front_free, right=right_free)

            time.sleep(0.05)

    finally:
        sim.stopSimulation()
