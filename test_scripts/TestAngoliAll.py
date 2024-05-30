from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import random
import math


def read_proximity_sensor(sensor):
    result, distance, detectedPoint, detected_object_handle, normalVector = sim.readProximitySensor(
        sensor)
    if result and detected_object_handle != -1:
        try:
            objectColor = sim.getObjectColor(
                detected_object_handle, 0, sim.colorcomponent_ambient_diffuse)
            return objectColor, distance
        except Exception as e:
            print(
                f"Errore nell'ottenere il colore dell'oggetto con handle {detected_object_handle}: {e}")
            return None, None
    else:
        return [1.0, 1.0, 1.0], None  # Assume white if no object is detected


def interpret_color(color):
    return color == [0.0, 0.0, 0.0]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def turn_left():
    global turning
    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]
    target_angle = normalize_angle(initial_angle + math.pi / 2)

    while turning:
        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]
        current_angle = normalize_angle(current_angle)
        # print(current_angle)
        angle = abs(normalize_angle(current_angle - target_angle)) <= 0.01

        sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
        sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
        black = interpret_color(read_proximity_sensor(front_sensor)[0])
        if black and angle:
            break
        # print("Turning left")

    turning = False


def turn_right():
    global turning
    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]
    target_angle = normalize_angle(initial_angle - math.pi / 2)

    sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
    sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
    print("Turning right")

    while True:
        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]
        current_angle = normalize_angle(current_angle)
        print(
            f"Turning right: initial_angle={initial_angle}, target_angle={target_angle}, current_angle={current_angle}")
        print(abs(normalize_angle(current_angle - target_angle)))
        if abs(normalize_angle(current_angle - target_angle)) < 0.001:
            break
    turning = False


def go_straight():
    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
    sim.setJointTargetVelocity(right_wheel_handle, base_speed)
    # print("Continuing straight")


def stop():
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)
    # print("Stopping")


def turn_randomly(left, right, front):
    global turning
    turning = True
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)
    # print("Intersection detected, choosing direction...")

    options = []
    if front:
        options.append("Front")
    if left:
        options.append("Left")
    if right:
        options.append("Right")

    if options:  # Se ci sono opzioni disponibili
        direction = random.choice(options)

        sim.setJointTargetVelocity(left_wheel_handle, base_speed)
        sim.setJointTargetVelocity(right_wheel_handle, base_speed)
        time.sleep(0.85)  # Avanza per 0.9 secondi
        sim.setJointTargetVelocity(left_wheel_handle, 0)
        sim.setJointTargetVelocity(right_wheel_handle, 0)
        time.sleep(intersection_delay)  # Tempo di attesa all'incrocio

        if direction == "Right":
            turning = True
            turn_right()
        elif direction == "Left":
            turning = True
            turn_left()
        elif direction == "Front":
            go_straight()


client = RemoteAPIClient()
sim = client.getObject("sim")

left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")
front_sensor = sim.getObjectHandle("/Front_Proximity_sensor")
left_sensor = sim.getObjectHandle("/Left_Proximity_sensor")
right_sensor = sim.getObjectHandle("/Right_Proximity_sensor")
robot_handle = sim.getObjectHandle("/PioneerP3DX")

sim.startSimulation()

base_speed = 0.5
turn_speed = 0.1
intersection_threshold = 0.1
intersection_delay = 1.0
turning = False

try:
    while True:
        front, front_distance = read_proximity_sensor(front_sensor)
        left, left_distance = read_proximity_sensor(left_sensor)
        right, right_distance = read_proximity_sensor(right_sensor)

        on_line_left = interpret_color(left)
        on_line_right = interpret_color(right)
        on_line_front = interpret_color(front)

        # print(turning)
        if turning is False:
            if on_line_front:
                if not on_line_left and not on_line_right:
                    go_straight()
                else:
                    turning = True
                    turn_randomly(left=on_line_left, front=False, right=False)
            else:
                if not on_line_left and not on_line_right:
                    stop()
                elif on_line_left and not on_line_right:
                    turning = True
                    turn_left()
                elif on_line_right and not on_line_left:
                    turning = True
                    turn_right()
                elif on_line_left and on_line_right:
                    turning = True
                    turn_randomly(left=True, front=False, right=True)

        time.sleep(0.05)

finally:
    sim.stopSimulation()
