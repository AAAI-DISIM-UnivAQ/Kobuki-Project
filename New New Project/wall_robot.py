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


def get_free():
    sens_left = sim.getObjectHandle("/ultrasonicSensor[0]")
    sens_front = sim.getObjectHandle("/ultrasonicSensor[4]")
    sens_right = sim.getObjectHandle("/ultrasonicSensor[7]")
    sens_front_free = is_free(sens_front)
    sens_left_free = is_free(sens_left)
    sens_right_free = is_free(sens_right)
    return sens_front_free, sens_left_free, sens_right_free


def get_robot_orientation():
    orientation = sim.getObjectOrientation(robot_handle, -1)
    return orientation[2]  # Restituisce l'angolo yaw


def normalize_angle(angle):
    normalized_angle = angle % (2 * math.pi)
    if normalized_angle >= math.pi:
        normalized_angle -= 2 * math.pi
    return normalized_angle


def go_straight():
    set_speeds(base_speed, base_speed)


# def turn_randomly(front, left, right):
def turn_randomly():
    time.sleep(1.9)
    set_speeds(0, 0)
    control_front, control_left, control_right = get_free()

    options = []
    if control_front:
        options.append("Front")
    if control_left:
        options.append("Left")
    if control_right:
        options.append("Right")
    # print("Opzioni:", str(options))
    direction = random.choice(options)

    if direction == "Right":
        # print("Right")
        turn_right()
    elif direction == "Left":
        # print("Left")
        turn_left()
    elif direction == "Front":
        # print("Front")
        go_straight()
        time.sleep(1.5)

    # sim.stopSimulation()


def turn_right():
    # current_angle = get_robot_orientation()
    # print("current", current_angle)
    # target_angle = current_angle + math.pi / 2
    # target_angle = math.pi / 2
    target_angle = find_target_angle("right")
    # print("target", target_angle)
    set_robot_orientation(target_angle, "right")
    go_straight()
    time.sleep(1.7)


def turn_left():
    # target_angle = math.pi / 2
    target_angle = find_target_angle("left")
    # print("target", target_angle)
    set_robot_orientation(target_angle, "left")
    go_straight()
    time.sleep(1.7)


def set_robot_orientation(target_angle, direction):
    actual_angle = get_robot_orientation()
    current_angle = normalize_angle(actual_angle)
    # print("current", current_angle)
    # print("diff", abs(target_angle - current_angle))
    # print("abs", abs(target_angle))
    if direction == "front":
        diff = abs(target_angle - current_angle)
    else:
        diff = abs(abs(target_angle) - abs(current_angle))
    # diff = abs((target_angle + math.pi) - (current_angle + math.pi))
    # print("current", current_angle + math.pi)
    # print("target", target_angle + math.pi)
    # print("diff", diff)
    # while abs(diff) > angle_tolerance:
    while diff > angle_tolerance:
        # print("abs", abs(target_angle))
        # print("current", current_angle + math.pi)
        # print("diff", diff)
        # if target_angle > current_angle:
        if direction == 'right' or direction == 'front':
            if diff > 0.8:
                # print("Rotazione normale")
                set_speeds(turn_speed, -turn_speed)
            elif 0.3 < diff < 0.8:
                # print("Rotazione più lenta")
                set_speeds(slow_turn_speed, -slow_turn_speed)
            else:
                # print("Rotazione molto lenta")
                set_speeds(more_slow_turn_speed, -more_slow_turn_speed)
        elif direction == 'left':
            if diff > 0.8:
                # print("Rotazione normale")
                set_speeds(-turn_speed, turn_speed)
            elif 0.3 < diff < 0.8:
                # print("Rotazione più lenta")
                set_speeds(-slow_turn_speed, slow_turn_speed)
            else:
                # print("Rotazione molto lenta")
                set_speeds(-more_slow_turn_speed, more_slow_turn_speed)
        current_angle = get_robot_orientation()
        if direction == "front":
            diff = abs(target_angle - current_angle)
        else:
            diff = abs(abs(target_angle) - abs(current_angle))
        # diff = abs((target_angle + math.pi) - (current_angle + math.pi))
        # time.sleep(0.01)  # Piccola pausa per evitare un loop troppo veloce

    # Ferma il robot dopo aver raggiunto l'angolo desiderato
    # set_speeds(0, 0)


def find_target_angle(direction):
    actual_angle = get_robot_orientation()
    current_angle = normalize_angle(actual_angle)
    # print("CURRENT:", current_angle)
    if direction == 'right':
        if -0.3 < current_angle < 0.3:
            return math.pi / 2
        elif -1.8 < current_angle < -1.2:
            return math.pi
        elif current_angle < -2.8 or current_angle > 2.8:
            return - math.pi / 2
        elif 1.2 < current_angle < 1.8:
            return 0
    if direction == 'left':
        if -0.3 < current_angle < 0.3:
            return - math.pi / 2
        elif -1.8 < current_angle < -1.2:
            return 0
        elif current_angle < -2.8 or current_angle > 2.8:
            return math.pi / 2
        elif 1.2 < current_angle < 1.8:
            return - math.pi


def go_back():
    actual_angle = get_robot_orientation()
    current_angle = normalize_angle(actual_angle)
    # print("CURRENT:", current_angle)
    target_angle_back = 0
    if -0.3 < current_angle < 0.3:
        target_angle_back = math.pi
    elif -1.8 < current_angle < -1.2:
        target_angle_back = math.pi / 2
    elif current_angle < -2.8 or current_angle > 2.8:
        target_angle_back = 0
    elif 1.2 < current_angle < 1.8:
        target_angle_back = - math.pi/2
    # print("target", target_angle_back)
    set_robot_orientation(target_angle_back, "front")
    go_straight()
    time.sleep(1.5)


class Coordinates:
    _x: int
    _y: int

    def __init__(self):
        self._x = 0
        self._y = 0

    def move(self):
        actual_angle = get_robot_orientation()
        current_angle = normalize_angle(actual_angle)
        if -0.3 < current_angle < 0.3:
            self._y += 1
        elif -1.8 < current_angle < -1.2:
            self._x += 1
        elif current_angle < -2.8 or current_angle > 2.8:
            self._y -= 1
        elif 1.2 < current_angle < 1.8:
            self._x -= 1


if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject("sim")

    robot_handle = sim.getObjectHandle("/PioneerP3DX")
    left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
    right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

    # left = sim.getObjectHandle("/ultrasonicSensor[0]")
    # front = sim.getObjectHandle("/ultrasonicSensor[4]")
    # right = sim.getObjectHandle("/ultrasonicSensor[7]")

    base_speed = 2.0
    turn_speed = 0.3
    slow_turn_speed = 0.2
    more_slow_turn_speed = 0.1
    MIN_DISTANCE = 0.5
    kp = 1.0
    angle_tolerance = 0.02  # Tolleranza per considerare l'angolo raggiunto

    sim.startSimulation()
    set_speeds(base_speed, base_speed)
    # time.sleep(0.5)

    # LEFT_DIST = get_distance(left)
    # RIGHT_DIST = get_distance(right)

    coord = Coordinates()
    crossroads = []

    try:
        while True:
            coord.move()
            print("X: " + str(coord._x), "Y: " + str(coord._y))
            # front_dist = get_distance(front)
            # left_dist = get_distance(left)
            # right_dist = get_distance(right)
            # print("left", left_dist)
            # print("right", right_dist)
            # print("front", front_dist)
            # front_free = is_free(front)
            # left_free = is_free(left)
            # right_free = is_free(right)
            front_free, left_free, right_free = get_free()
            # print("front", front_free, "left", left_free, "right", right_free)

            if front_free:
                if not left_free and not right_free:
                    # go_straight(left_dist, right_dist)
                    go_straight()
                else:
                    print("Incrocio")
                    print("Incroci incontrati:", str(crossroads))
                    # turn_randomly(front_free, left_free, right_free)
                    crossroads.append((coord._x, coord._y))
                    turn_randomly()
            else:
                if not left_free and not right_free:
                    print("Vicolo cieco")
                    go_back()
                # else:
                    # print("Incrocio T o curva")
                    # turn_randomly(front_free, left_free, right_free)
                    # turn_randomly()

    finally:
        sim.stopSimulation()
