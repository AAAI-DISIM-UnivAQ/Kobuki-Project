from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import random
import time
import math


def set_speeds(left_speed, right_speed):
    sim.setJointTargetVelocity(left_wheel_handle, left_speed)
    sim.setJointTargetVelocity(right_wheel_handle, right_speed)


def get_distance(sensor):
    _, dist, _, _, _ = sim.readProximitySensor(sensor)
    return dist


def is_free(sensor):
    _, dist, _, _, _ = sim.readProximitySensor(sensor)
    return dist == 0 or dist > MIN_DISTANCE


def get_robot_orientation():
    orientation = sim.getObjectOrientation(robot_handle, -1)
    return orientation[2]  # Restituisce l'angolo yaw


def normalize_angle(angle):
    """
    Normalizza l'angolo in modo che sia compreso tra -pi e pi.
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    print("Normalizzato", angle)
    return angle


def angular_difference(angle1, angle2):
    """
    Calcola la differenza tra due angoli, tenendo conto del wrapping.
    """
    print("target", angle1, "current", angle2)
    diff = angle1 - angle2
    return normalize_angle(diff)


def go_straight(left_dist, front_dist):
    error = front_dist - left_dist
    adjustment = kp * error
    left_speed = base_speed - adjustment
    right_speed = base_speed + adjustment
    set_speeds(left_speed, right_speed)


def normal_go_straight():
    set_speeds(base_speed, base_speed)


def turn_randomly(front, left, right):
    time.sleep(1.9)
    set_speeds(0, 0)

    options = []
    if front:
        options.append("Front")
    if left:
        options.append("Left")
    if right:
        options.append("Right")
    direction = random.choice(options)

    if direction == "Right":
        print("Right")
        turn_right()
    elif direction == "Left":
        print("Left")
        turn_left()
    elif direction == "Front":
        print("Front")
        normal_go_straight()
        time.sleep(1.5)

    # sim.stopSimulation()


def turn_right():
    # current_angle = get_robot_orientation()
    # print("current", current_angle)
    # target_angle = current_angle + math.pi / 2
    # target_angle = math.pi / 2
    target_angle = find_target_angle("right")
    print("target", target_angle)
    set_robot_orientation(target_angle, "right")
    normal_go_straight()
    time.sleep(1.5)


def turn_left():
    # target_angle = math.pi / 2
    target_angle = find_target_angle("left")
    print("target", target_angle)
    set_robot_orientation(target_angle, "left")
    normal_go_straight()
    time.sleep(1.5)


def set_robot_orientation(target_angle, direction):
    current_angle = get_robot_orientation()
    # print("current", current_angle)
    # print("diff", abs(target_angle - current_angle))
    # print("abs", abs(target_angle))
    diff = abs(abs(target_angle) - abs(current_angle))
    # diff = angular_difference(target_angle, current_angle)
    # print("current", current_angle)
    print("diff", diff)
    while abs(diff) > angle_tolerance:
        # print("abs", abs(target_angle))
        print("current", current_angle)
        print("diff", diff)
        # if target_angle > current_angle:
        if direction == 'right' or direction == 'front':
            if diff > 0.8:
                print("Rotazione normale")
                set_speeds(turn_speed, -turn_speed)
            elif 0.3 < diff < 0.8:
                print("Rotazione più lenta")
                set_speeds(slow_turn_speed, -slow_turn_speed)
            else:
                print("Rotazione molto lenta")
                set_speeds(more_slow_turn_speed, -more_slow_turn_speed)
        elif direction == 'left':
            if diff > 0.8:
                print("Rotazione normale")
                set_speeds(-turn_speed, turn_speed)
            elif 0.3 < diff < 0.8:
                print("Rotazione più lenta")
                set_speeds(-slow_turn_speed, slow_turn_speed)
            else:
                print("Rotazione molto lenta")
                set_speeds(-more_slow_turn_speed, more_slow_turn_speed)
        current_angle = get_robot_orientation()
        diff = abs(abs(target_angle) - abs(current_angle))
        # diff = angular_difference(target_angle, current_angle)
        # time.sleep(0.01)  # Piccola pausa per evitare un loop troppo veloce

    # Ferma il robot dopo aver raggiunto l'angolo desiderato
    # set_speeds(0, 0)


def find_target_angle(direction):
    current_angle = get_robot_orientation()
    print("CURRENT:", current_angle)
    if direction == 'right':
        if -0.3 < current_angle < 0.3:
            return math.pi / 2
        elif -1.8 < current_angle < -1.2:
            return math.pi
        elif current_angle < -2.8 or current_angle > 2.8:
            return - math.pi
        elif 1.2 < current_angle < 1.8:
            return 0
    if direction == 'left':
        if -0.3 < current_angle < 0.3:
            return - math.pi / 2
        elif -1.8 < current_angle < -1.2:
            return 0
        elif current_angle < -2.8 or current_angle > 2.8:
            return math.pi
        elif 1.2 < current_angle < 1.8:
            return - math.pi


def go_back():
    current_angle = get_robot_orientation()
    target_angle_back = 0
    if -0.3 < current_angle < 0.3:
        target_angle_back = math.pi
    elif -1.8 < current_angle < -1.2:
        target_angle_back = - math.pi / 2
    elif current_angle < -2.8 or current_angle > 2.8:
        target_angle_back = 0
    elif 1.2 < current_angle < 1.8:
        target_angle_back = math.pi/2
    print("target", target_angle_back)
    set_robot_orientation(target_angle_back, "front")
    normal_go_straight()
    time.sleep(1.5)




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
    slow_turn_speed = 0.2
    more_slow_turn_speed = 0.1
    MIN_DISTANCE = 0.4
    kp = 1.0
    angle_tolerance = 0.02  # Tolleranza per considerare l'angolo raggiunto

    sim.startSimulation()
    set_speeds(base_speed, base_speed)
    # time.sleep(0.5)

    # LEFT_DIST = get_distance(left)
    # RIGHT_DIST = get_distance(right)

    try:
        while True:
            front_dist = get_distance(front)
            left_dist = get_distance(left)
            right_dist = get_distance(right)
            # print("left", left_dist)
            # print("right", right_dist)
            front_free = is_free(front)
            left_free = is_free(left)
            right_free = is_free(right)
            # print("front", front_free, "left", left_free, "right", right_free)

            if front_free:
                if not left_free and not right_free:
                    # go_straight(left_dist, right_dist)
                    normal_go_straight()
                else:
                    turn_randomly(front_free, left_free, right_free)
            else:
                if not left_free and not right_free:
                    go_back()
                else:
                    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
                    turn_randomly(front_free, left_free, right_free)

    finally:
        sim.stopSimulation()
