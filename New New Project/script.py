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


def turn_randomly(dist, left, right, front):
    print("Intersection detected, choosing direction...")

    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
    sim.setJointTargetVelocity(right_wheel_handle, base_speed)

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


def stay_in_center(left, right):
    kp = 0.5
    kd = 0.1
    threshold = 0.1
    base_speed = 1.0

    previous_error = 0.0

    while True:
        error = left - right
        derivative = error - previous_error

        # Calcolare la correzione con controllo PD
        correction = kp * error + kd * derivative

        # Applicare la correzione solo se l'errore è sopra la soglia
        if abs(error) > threshold:
            if error > 0:
                # Correzione a sinistra
                sim.setJointTargetVelocity(
                    left_wheel_handle, base_speed - correction)
                sim.setJointTargetVelocity(
                    right_wheel_handle, base_speed + correction)
                print("Correzione a sinistra")
            else:
                # Correzione a destra
                sim.setJointTargetVelocity(
                    left_wheel_handle, base_speed + correction)
                sim.setJointTargetVelocity(
                    right_wheel_handle, base_speed - correction)
                print("Correzione a destra")
        else:
            # Procedere dritto se l'errore è sotto la soglia
            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)
            print("Procedo dritto")

            # Aggiungere un breve ritardo per stabilizzare il movimento
            time.sleep(0.5)

        # Aggiornare l'errore precedente
        previous_error = error


def get_distance(sensor):
    _, dist, _, _, _ = sim.readProximitySensor(sensor)
    return dist


def is_free(sensor):
    _, dist, _, _, _ = sim.readProximitySensor(sensor)
    # print(sensor, dist)
    return dist == 0 or dist > MIN_DISTANCE


if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject("sim")

    robot_handle = sim.getObjectHandle("/PioneerP3DX")
    left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
    right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

    left = sim.getObjectHandle("/ultrasonicSensor[0]")
    front = sim.getObjectHandle("/ultrasonicSensor[4]")
    right = sim.getObjectHandle("/ultrasonicSensor[7]")

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

            front_dist = get_distance(front)
            left_dist = get_distance(left)
            right_dist = get_distance(right)
            # print(front_dist, left_dist, right_dist)

            # stay_in_center(left_dist, right_dist)

            if front_free:
                if not left_free and not right_free:
                    # print("front, not left, not right")
                    go_straight()
                else:
                    # print("front, left, right")
                    turn_randomly(left_dist + right_dist, left=left_free,
                                  front=front_free, right=right_free)
            else:
                if not left_free and not right_free:
                    # print("not front, not left, not right")
                    stop()
                elif left_free and not right_free:
                    # print("not frotn, left, not right")
                    turn_left()
                elif right_free and not left_free:
                    # print("not front, not left, right")
                    turn_right()
                elif left_free and right_free:
                    # print("not front, left, right")
                    turn_randomly(left_dist + right_dist, left=left_free,
                                  front=front_free, right=right_free)

            time.sleep(0.05)

    finally:
        sim.stopSimulation()
