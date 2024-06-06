from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import random
import time
import math


def get_distance(sensor):
    _, dist, _, _, _ = sim.readProximitySensor(sensor)
    return dist


def is_free(sensor):
    _, dist, _, _, _ = sim.readProximitySensor(sensor)
    return dist == 0 or dist > MIN_DISTANCE


def get_robot_orientation():
    orientation = sim.getObjectOrientation(robot_handle, -1)
    return orientation[2]  # Restituisce l'angolo yaw


def go_straight(left_dist, front_dist):
    error = front_dist - left_dist
    adjustment = kp * error
    left_speed = base_speed - adjustment
    right_speed = base_speed + adjustment
    sim.setJointTargetVelocity(left_wheel_handle, left_speed)
    sim.setJointTargetVelocity(right_wheel_handle, right_speed)


def normal_go_straight():
    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
    sim.setJointTargetVelocity(right_wheel_handle, base_speed)


def turn_randomly(front, left, right):
    time.sleep(1.5)
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)
    options = []
    if front:
        options.append("Front")
    if left:
        options.append("Left")
    if right:
        options.append("Right")
    direction = random.choice(options)

    if direction == "Right":
        turn_right()
        print("Right")
    elif direction == "Left":
        # turn_left()
        print("Left")
    elif direction == "Front":
        print("Front")
        # normal_go_straight()
        # time.sleep(1.5)

    # sim.stopSimulation()


def turn_right():
    # current_angle = get_robot_orientation()
    # print("current", current_angle)
    # target_angle = current_angle + math.pi / 2
    target_angle = math.pi / 2
    print("target", target_angle)
    set_robot_orientation(target_angle)
    normal_go_straight()


def set_robot_orientation(target_angle):
    current_angle = get_robot_orientation()
    # print("current", current_angle)
    # print("diff", abs(target_angle - current_angle))
    while abs(target_angle - abs(current_angle)) > angle_tolerance:
        print("current", current_angle)
        print("diff", abs(target_angle - abs(current_angle)))
        if target_angle > current_angle:
            sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
            sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
        else:
            sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
            sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
        current_angle = get_robot_orientation()
        time.sleep(0.01)  # Piccola pausa per evitare un loop troppo veloce

    # Ferma il robot dopo aver raggiunto l'angolo desiderato
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)


if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject("sim")

    robot_handle = sim.getObjectHandle("/PioneerP3DX")
    left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
    right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

    left = sim.getObjectHandle("/ultrasonicSensor[0]")
    front = sim.getObjectHandle("/ultrasonicSensor[4]")
    right = sim.getObjectHandle("/ultrasonicSensor[7]")

    base_speed = 2.0
    turn_speed = 0.3
    MIN_DISTANCE = 0.4
    kp = 1.0
    angle_tolerance = 0.01  # Tolleranza per considerare l'angolo raggiunto

    sim.startSimulation()
    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
    sim.setJointTargetVelocity(right_wheel_handle, base_speed)
    # time.sleep(0.5)

    # LEFT_DIST = get_distance(left)
    # RIGHT_DIST = get_distance(right)

    try:
        while True:
            front_dist = get_distance(front)
            left_dist = get_distance(left)
            right_dist = get_distance(right)
            print("left", left_dist)
            print("right", right_dist)
            front_free = is_free(front)
            left_free = is_free(left)
            right_free = is_free(right)
            print("front", front_free, "left", left_free, "right", right_free)

            if front_free:
                if not left_free and not right_free:
                    # go_straight(left_dist, right_dist)
                    normal_go_straight()
                else:
                    turn_randomly(front_free, left_free, right_free)

    finally:
        sim.stopSimulation()





