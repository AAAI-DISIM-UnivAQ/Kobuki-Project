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

    if (control_front + control_left + control_right) >= 2:
        # INCROCIO
        cross = find_cross(crossroads, coord)
        actual = cross.define_direction()
        print("actual", actual)
        if len(cross.directions) == 0:
            print("Init directions")
            cross.initialize_directions(control_front, control_left, control_right)
        else:
            cross.reverse_direction_status(actual)
        print("Coord: " + str(cross.x) + ", " + str(cross.y))
        print("Directions: " + str(cross.directions))

        options = cross.get_true_directions()
        print("Available: ", str(options))
        choice = random.choice(options)
        print("Choice: " + str(choice))
        cross.set_direction_status(choice)
        print("Direction status: " + str(cross.directions))
        # actual = cross.define_direction()
        if choice == actual:
            go_straight()
            coord.move(30)
            time.sleep(1.9)
        elif ((actual == "nord" and choice == "est") or
              (actual == "sud" and choice == "ovest") or
              (actual == "est" and choice == "sud") or
              (actual == "ovest" and choice == "nord")):
            turn_right()
        elif ((actual == "nord" and choice == "ovest") or
              (actual == "sud" and choice == "est") or
              (actual == "est" and choice == "nord") or
              (actual == "ovest" and choice == "sud")):
            turn_left()
    else:
        # SVOLTE O BUG
        if control_front:
            go_straight()
            coord.move(30)
            time.sleep(1.9)
        elif control_left:
            turn_left()
        elif control_right:
            turn_right()

    """
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
        coord.move(30)
        time.sleep(1.9)

    # sim.stopSimulation()
    """


def turn_right():
    # current_angle = get_robot_orientation()
    # print("current", current_angle)
    # target_angle = current_angle + math.pi / 2
    # target_angle = math.pi / 2
    target_angle = find_target_angle("right")
    # print("target", target_angle)
    set_robot_orientation(target_angle, "right")
    go_straight()
    coord.move(30)
    time.sleep(1.9)


def turn_left():
    # target_angle = math.pi / 2
    target_angle = find_target_angle("left")
    # print("target", target_angle)
    set_robot_orientation(target_angle, "left")
    go_straight()
    coord.move(30)
    time.sleep(1.9)


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
    # time.sleep(1.7)


def is_far_enough(x, y, crossroads, threshold=30):
    for cross in crossroads:
        if abs(cross.x - x) <= threshold and abs(cross.y - y) <= threshold:
            return False
    return True


def find_cross(crossroads_list, coord, threshold=30):
    for cross in crossroads_list:
        if abs(cross.x - coord.x) <= threshold and abs(cross.y - coord.y) <= threshold:
            return cross
    return None


class Coordinates:
    x: int
    y: int

    def __init__(self):
        self.x = 0
        self.y = 0

    def move(self, d):
        actual_angle = get_robot_orientation()
        current_angle = normalize_angle(actual_angle)
        if -0.3 < current_angle < 0.3:
            self.y += d
        elif -1.8 < current_angle < -1.2:
            self.x += d
        elif current_angle < -2.8 or current_angle > 2.8:
            self.y -= d
        elif 1.2 < current_angle < 1.8:
            self.x -= d


class Crossroad:
    x: int
    y: int
    # coordinates: Coordinates
    directions: list

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.directions = []

    def initialize_directions(self, front, left, right):
        actual_dir = self.define_direction()
        if actual_dir == "nord":
            # self.directions.append([math.pi, False])
            self.directions.append(["sud", False])
            if front:
                # self.directions.append([0.0, True])
                self.directions.append(["nord", True])
            if left:
                # self.directions.append([- math.pi / 2, True])
                self.directions.append(["ovest", True])
            if right:
                # self.directions.append([math.pi / 2, True])
                self.directions.append(["est", True])
        elif actual_dir == "est":
            # self.directions.append([- math.pi / 2, False])
            self.directions.append(["ovest", False])
            if front:
                # self.directions.append([math.pi / 2, True])
                self.directions.append(["est", True])
            if left:
                # self.directions.append([0.0, True])
                self.directions.append(["nord", True])
            if right:
                # self.directions.append([math.pi, True])
                self.directions.append(["sud", True])
        elif actual_dir == "ovest":
            # self.directions.append([math.pi / 2, False])
            self.directions.append(["est", False])
            if front:
                # self.directions.append([- math.pi / 2, True])
                self.directions.append(["ovest", True])
            if left:
                # self.directions.append([math.pi, True])
                self.directions.append(["sud", True])
            if right:
                # self.directions.append([0.0, True])
                self.directions.append(["nord", True])
        elif actual_dir == "sud":
            # self.directions.append([0.0, False])
            self.directions.append(["nord", False])
            if front:
                # self.directions.append([math.pi, True])
                self.directions.append(["sud", True])
            if left:
                # self.directions.append([math.pi / 2, True])
                self.directions.append(["est", True])
            if right:
                # self.directions.append([- math.pi / 2, True])
                self.directions.append(["ovest", True])

    def define_direction(self):
        actual_angle = get_robot_orientation()
        current_angle = normalize_angle(actual_angle)
        if -0.3 < current_angle < 0.3:
            return "nord"
        elif -1.8 < current_angle < -1.2:
            return "est"
        elif current_angle < -2.8 or current_angle > 2.8:
            return "sud"
        elif 1.2 < current_angle < 1.8:
            return "ovest"

    def get_true_directions(self):
        """
        direction_map = {
            0.0: "nord",
            math.pi / 2: "est",
            math.pi: "sud",
            -math.pi / 2: "ovest"
        }
        return [direction_map[direction[0]] for direction in self.directions if direction[1] is True]
        """
        return [direction[0] for direction in self.directions if direction[1] is True]

    def set_direction_status(self, direction_name):
        """
        direction_map = {
            "nord": 0.0,
            "est": math.pi / 2,
            "sud": math.pi,
            "ovest": -math.pi / 2
        }
        target_angle = direction_map[direction_name]
        for direction in self.directions:
            if direction[0] == target_angle:
                direction[1] = False
                break
        """
        for direction in self.directions:
            if direction[0] == direction_name:
                direction[1] = False
                break

        self.reset()

    def reset(self):
        # Controllo se tutte le direzioni sono false
        if all(direction[1] is False for direction in self.directions):
            self.directions[0][1] = True

    def reverse_direction_status(self, direction_name):
        opposite_direction = {
            "nord": "sud",
            "est": "ovest",
            "sud": "nord",
            "ovest": "est"
        }
        if direction_name in opposite_direction:
            opposite_name = opposite_direction[direction_name]
            for direction in self.directions:
                if direction[0] == opposite_name:
                    direction[1] = False
                    break

            self.reset()



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

    # start_time = time.time()
    # elapsed_time = 0

    # LEFT_DIST = get_distance(left)
    # RIGHT_DIST = get_distance(right)

    coord = Coordinates()
    # cross = Crossroad()
    crossroads = []

    try:
        while True:
        # while elapsed_time < 1.9:
            coord.move(1)
            # cross.move(1)
            print("X: " + str(coord.x), "Y: " + str(coord.y))
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
                    coord.move(30)
                    # cross.move(30)
                    if is_far_enough(coord.x, coord.y, crossroads):
                        print("Nuovo incrocio")
                        # crossroads.append((coord.x, coord.y))
                        crossroads.append(Crossroad(coord.x, coord.y))
                    else:
                        print("Incrocio già incontrato")
                    print("Incroci incontrati:", str(crossroads))
                    # turn_randomly(front_free, left_free, right_free)
                    # crossroads.append((coord._x, coord._y))
                    turn_randomly()
            else:
                if not left_free and not right_free:
                    print("Vicolo cieco")
                    go_back()
                # else:
                    # print("Incrocio T o curva")
                    # turn_randomly(front_free, left_free, right_free)
                    # turn_randomly()
            # elapsed_time = time.time() - start_time

    finally:
        sim.stopSimulation()
