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


def go_straight(left_dist, front_dist):
    error = front_dist - left_dist
    adjustment = kp * error
    left_speed = base_speed - adjustment
    right_speed = base_speed + adjustment
    set_speeds(left_speed, right_speed)


def normal_go_straight():
    set_speeds(base_speed, base_speed)


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
    print("Opzioni:", str(options))
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


def turn_right():
    target_angle = find_target_angle("right")
    print("target", target_angle)
    set_robot_orientation(target_angle, "right")
    normal_go_straight()
    coordinates.set_direction("right")
    time.sleep(1.5)


def turn_left():
    target_angle = find_target_angle("left")
    print("target", target_angle)
    set_robot_orientation(target_angle, "left")
    normal_go_straight()
    coordinates.set_direction("left")
    time.sleep(1.5)


def set_robot_orientation(target_angle, direction):
    actual_angle = get_robot_orientation()
    current_angle = normalize_angle(actual_angle)
    print("current", current_angle)
    diff = abs(abs(target_angle) - abs(current_angle))
    while diff > angle_tolerance:
        if direction == 'right' or direction == 'front':
            if diff > 0.8:
                set_speeds(turn_speed, -turn_speed)
            elif 0.3 < diff < 0.8:
                set_speeds(slow_turn_speed, -slow_turn_speed)
            else:
                set_speeds(slower_turn_speed, -slower_turn_speed)
        elif direction == 'left':
            if diff > 0.8:
                set_speeds(-turn_speed, turn_speed)
            elif 0.3 < diff < 0.8:
                set_speeds(-slow_turn_speed, slow_turn_speed)
            else:
                set_speeds(-slower_turn_speed, slower_turn_speed)
        current_angle = get_robot_orientation()
        diff = abs(abs(target_angle) - abs(current_angle))


def find_target_angle(direction):
    actual_angle = get_robot_orientation()
    current_angle = normalize_angle(actual_angle)
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
    target_angle_back = 0
    if -0.3 < current_angle < 0.3:
        target_angle_back = math.pi
    elif -1.8 < current_angle < -1.2:
        target_angle_back = - math.pi / 2
    elif current_angle < -2.8 or current_angle > 2.8:
        target_angle_back = 0
    elif 1.2 < current_angle < 1.8:
        target_angle_back = math.pi/2
    set_robot_orientation(target_angle_back, "front")
    normal_go_straight()
    coordinates.set_direction("back")
    time.sleep(1.5)


crossroads = []


class CoordinatesDS:

    _x: int
    _y: int
    _prev: str
    _direction: str
    _id: int

    def __init__(self, direction):
        self._x = 0
        self._y = 0
        self._direction = direction
        self._id = 0

    def set_direction(self, direction):
        tmp = self._direction
        self._direction = direction
        self._prev = tmp

    def save_crossroad(self):
        crossroads.append({self.id, (self._x, self._y)})
        self.id += 1

    def moving(self):
        angle = math.degrees(normalize_angle(get_robot_orientation()))
        norm_45 = normalize_angle(45)
        norm_135 = normalize_angle(135)

        if -norm_45 <= angle < norm_45:
            self._prev = "front"
        elif norm_45 <= angle < norm_135:
            self._prev = "right"
        elif -norm_135 <= angle < -norm_45:
            self._prev = "left"
        else:
            self._prev = "back"

        if self._prev == "front":
            match self._direction:
                case "front":
                    self._y += 1
                case "right":
                    self._x += 1
                case "back":
                    self._y -= 1
                case "left":
                    self._x -= 1
        elif self._prev == "right":
            match self._direction:
                case "front":
                    self._x += 1
                case "right":
                    self._y -= 1
                case "back":
                    self._x -= 1
                case "left":
                    self._y += 1
        elif self._prev == "back":
            match self._direction:
                case "front":
                    self._y -= 1
                case "right":
                    self._x -= 1
                case "left":
                    self._x += 1
                case "back":
                    self._y += 1
        elif self._prev == "left":
            match self._direction:
                case "front":
                    self._y -= 1
                case "right":
                    self._y += 1
                case "back":
                    self._x += 1
                case "left":
                    self._y -= 1


if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject("sim")

    robot_handle = sim.getObjectHandle("/PioneerP3DX")
    left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
    right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

    base_speed = 2.0
    turn_speed = 0.3
    slow_turn_speed = 0.2
    slower_turn_speed = 0.1
    MIN_DISTANCE = 0.5
    kp = 1.0
    angle_tolerance = 0.02  # Tolleranza per considerare l'angolo raggiunto

    sim.startSimulation()
    set_speeds(base_speed, base_speed)

    coordinates = CoordinatesDS("front")

    try:
        while True:

            coordinates.moving()
            print(coordinates._x, coordinates._y,
                  coordinates._direction, coordinates._prev)

            front_free, left_free, right_free = get_free()

            if front_free:
                if not left_free and not right_free:
                    normal_go_straight()
                else:
                    print("Incrocio")
                    turn_randomly()
            else:
                if not left_free and not right_free:
                    print("Vicolo cieco")
                    go_back()
                else:
                    print("Incrocio T o curva")
                    turn_randomly()

    finally:
        sim.stopSimulation()
