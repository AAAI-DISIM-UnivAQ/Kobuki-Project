import paho.mqtt.client as mqtt
import math
import time
import random
from Crossroad import Crossroad

MAX_CORRECTION = 50
ANGLE_TOLERANCE = 0.02


class Controller:
    _my_possible_perceptions: list
    _my_name: str
    _old_action: str
    _old_perception: str
    _free_directions: dict
    _direction: float
    _rotating: bool
    _target_angle: float
    _rotation_sense: str
    _target: bool

    def __init__(self, name, possible_perceptions, old_action):
        assert isinstance(name, str) and isinstance(possible_perceptions, list)
        self._my_name = name
        self._my_possible_perceptions = possible_perceptions
        self._old_action = old_action
        self._old_perception = ""
        self._free_directions = {
            "front": True,
            "left": False,
            "right": False
        }
        self._direction = 0.0
        self._rotating = False
        self._target_angle = 0.0
        self._rotation_sense = ""
        self.rotation_done = False
        self.waiting_update_direction = False
        self.update_direction = {
            "front": False,
            "left": False,
            "right": False,
            "x": False,
            "y": False
        }
        self.crossroads = []
        self.position = {
            "x": 0.0,
            "y": 0.0
        }
        self.dx_sx = False
        self._target = False

    def control_directions(self):
        global client_mqtt
        front = self._free_directions["front"]
        left = self._free_directions["left"]
        right = self._free_directions["right"]

        # print("Rotating", self._rotating)
        # print("POSIZIONE", str(controller.position))
        # print("Rotazione", self._rotating)
        # print("Front:", str(front), "Left:", str(left), "Right:", str(right))
        """
        print("Incroci incontrati", len(self.crossroads))
        if len(self.crossroads) > 0:
            i = 1
            for crossroad in self.crossroads:
                print(f"Incrocio {i}: {crossroad.x} - {crossroad.y}")
                i += 1
        """

        if self.update_direction["front"] and self.update_direction["left"] and self.update_direction["right"] \
                and self.update_direction["x"] and self.update_direction["y"]:
            self.waiting_update_direction = False

        if not self._rotating:
            if self.rotation_done:
                if front and not left and not right:
                    print("FINISH ROTATION DONE")
                    self.rotation_done = False
                else:
                    print("ROTATION DONE")
                    return "go"
            else:
                if self._old_perception == "cross" and not self.waiting_update_direction:
                    if (front + left + right) >= 2:
                        if self.is_far_enough(controller.position["x"], controller.position["y"], controller.crossroads):
                            print("Nuovo Incrocio !!!")
                            self.crossroads.append(Crossroad(controller.position["x"], controller.position["y"]))
                        else:
                            print("Incrocio già incontrato !!!")

                        actual_cross = self.find_cross(self.crossroads, self.position)
                        print("Incrocio Considerato:", actual_cross.x, actual_cross.y)
                        actual_dir = actual_cross.define_direction(controller._direction)
                        if len(actual_cross.directions) == 0:
                            print("Init directions")
                            actual_cross.initialize_directions(front, left, right, self._direction)
                        else:
                            actual_cross.reverse_direction_status(actual_dir)
                        print("Directions:", str(actual_cross.directions))

                        print("SCELTA DELLA DIREZIONE")
                        self._rotating = True
                        for el in self.update_direction.keys():
                            self.update_direction[el] = False
                        return self.turn_randomly(actual_cross, actual_dir)
                    else:
                        print("CURVA")
                        self._rotating = True
                        for el in self.update_direction.keys():
                            self.update_direction[el] = False
                        if left:
                            return self.turn_left()
                        elif right:
                            return self.turn_right()

                elif self._old_perception == "cross" and self.waiting_update_direction:
                    # print("WAITING UPDATE", str(self.update_direction))
                    return "cross"
                else:
                    if front:
                        if not left and not right:
                            # strada dritta
                            return "go"
                        else:
                            # incrocio a 4
                            print("INCROCIO INCONTRATO")
                            client_mqtt.disconnect()
                            time.sleep(1.9)
                            client_mqtt.reconnect()
                            self.waiting_update_direction = True
                            return "cross"
                    else:
                        """
                        if left and right:
                            # incrocio a T
                            print("INCROCIO A T")
                            # client_mqtt.disconnect()
                            # time.sleep(1.9)
                            # client_mqtt.reconnect()
                            # self._rotating = True
                            # return self.turn_randomly()
                        elif right:
                            # curva a destra
                            print("CURVA DX")
                            # return self.turn_right()
                        elif left:
                            # curva a sinistra
                            print("CURVA SX")
                            # return self.turn_left()
                        """
                        if not left and not right:
                            # vicolo cieco
                            print("VICOLO CIECO")
                            self._rotating = True
                            return self.go_back()
                        else:
                            return "undetermined"  # Opzionale, per gestire altri casi se necessario
        else:
            return self.set_robot_orientation(self._target_angle, self._rotation_sense, self.dx_sx)

    def go_back(self):
        actual_angle = self._direction
        current_angle = self.normalize_angle(actual_angle)
        self.dx_sx = False
        if -0.3 < current_angle < 0.3:
            self._target_angle = math.pi
        elif -1.8 < current_angle < -1.2:
            self._target_angle = math.pi / 2
            self.dx_sx = True
        elif current_angle < -2.8 or current_angle > 2.8:
            self._target_angle = 0
        elif 1.2 < current_angle < 1.8:
            self._target_angle = - math.pi / 2
            self.dx_sx = True
        self._rotation_sense = "front"
        return self.set_robot_orientation(self._target_angle, self._rotation_sense, self.dx_sx)

    def normalize_angle(self, angle):
        normalized_angle = angle % (2 * math.pi)
        if normalized_angle >= math.pi:
            normalized_angle -= 2 * math.pi
        return normalized_angle

    def set_robot_orientation(self, target_angle, dir, dx_sx):
        actual_angle = self._direction
        current_angle = self.normalize_angle(actual_angle)
        # diff = abs(abs(target_angle) - abs(current_angle))
        if dir == "front" and dx_sx:
            diff = abs(target_angle - current_angle)
        else:
            diff = abs(abs(target_angle) - abs(current_angle))
        if diff > ANGLE_TOLERANCE:
            if dir == 'right' or dir == 'front':
                if diff > 0.8:
                    return "turn_right"
                elif 0.3 < diff < 0.8:
                    return "turn_right_slow"
                else:
                    return "turn_right_more_slow"
            elif dir == 'left':
                if diff > 0.8:
                    return "turn_left"
                elif 0.3 < diff < 0.8:
                    return "turn_left_slow"
                else:
                    return "turn_left_more_slow"
        else:
            self._rotating = False
            self.rotation_done = True
            self.dx_sx = False
            return "go"

    def turn_right(self):
        self.find_target_angle("right")
        self._rotation_sense = "right"
        return self.set_robot_orientation(self._target_angle, self._rotation_sense, self.dx_sx)

    def turn_left(self):
        self.find_target_angle("left")
        self._rotation_sense = "left"
        return self.set_robot_orientation(self._target_angle, self._rotation_sense, self.dx_sx)

    def turn_randomly(self, cross, actual_dir):
        # a = []
        # for direction, is_free in self._free_directions.items():
            # if is_free:
                # a.append(direction)
        # print("DIREZIONI DISPONIBILI", a)
        # rand = random.choice(a)
        options = cross.get_true_directions()
        print("Available: ", str(options))
        choice = random.choice(options)
        print("DIREZIONE SCELTA", choice)
        cross.set_direction_status(choice)
        print("Direction status: " + str(cross.directions))

        if choice == actual_dir:
            self._rotating = False
            self.rotation_done = True  # usare per far andare dritto il robot
            return "go"
        elif ((actual_dir == "nord" and choice == "est") or
              (actual_dir == "sud" and choice == "ovest") or
              (actual_dir == "est" and choice == "sud") or
              (actual_dir == "ovest" and choice == "nord")):
            return self.turn_right()
        elif ((actual_dir == "nord" and choice == "ovest") or
              (actual_dir == "sud" and choice == "est") or
              (actual_dir == "est" and choice == "nord") or
              (actual_dir == "ovest" and choice == "sud")):
            return self.turn_left()

        # if rand == "front":
            # self._rotating = False
            # self.rotation_done = True  # usare per far andare dritto il robot
            # return "go"
        # elif rand == "right":
            # return self.turn_right()
        # elif rand == "left":
            # return self.turn_left()

    def find_target_angle(self, direction):
        actual_angle = self._direction
        current_angle = self.normalize_angle(actual_angle)
        if direction == 'right':
            if -0.3 < current_angle < 0.3:
                self._target_angle = math.pi / 2
            elif -1.8 < current_angle < -1.2:
                self._target_angle = math.pi
            elif current_angle < -2.8 or current_angle > 2.8:
                self._target_angle = - math.pi / 2
            elif 1.2 < current_angle < 1.8:
                self._target_angle = 0
        if direction == 'left':
            if -0.3 < current_angle < 0.3:
                self._target_angle = - math.pi / 2
            elif -1.8 < current_angle < -1.2:
                self._target_angle = 0
            elif current_angle < -2.8 or current_angle > 2.8:
                self._target_angle = math.pi / 2
            elif 1.2 < current_angle < 1.8:
                self._target_angle = - math.pi

    def is_far_enough(self, x, y, crossroads, threshold=0.4):
        for cross in crossroads:
            if abs(cross.x - x) <= threshold and abs(cross.y - y) <= threshold:
                return False
        return True

    def find_cross(self, crossroads_list, coord, threshold=0.4):
        for cross in crossroads_list:
            if abs(cross.x - coord["x"]) <= threshold and abs(cross.y - coord["y"]) <= threshold:
                return cross
        return None


def update_direction(name, val):
    if val == "True":
        controller._free_directions[name] = True
    else:
        controller._free_directions[name] = False
    if controller.waiting_update_direction:
        controller.update_direction[name] = True


def update_position(name, val):
    controller.position[name] = float(val)
    if controller.waiting_update_direction:
        controller.update_direction[name] = True

def update_target(val):
    if val == "True":
        controller._target = True
    else:
        controller._target = False
    print("Target:", controller._target)

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("perception/#")


def on_message(client, userdata, msg):
    perception_name = msg.topic.split("/")[1]
    message_value = msg.payload.decode("utf-8")

    match perception_name:
        case "front":
            update_direction(perception_name, message_value)
        case "left":
            update_direction(perception_name, message_value)
        case "right":
            update_direction(perception_name, message_value)
        case "green":
            update_target(message_value)
        case "orientation":
            controller._direction = float(message_value)
        case "position_x":
            update_position("x", message_value)
        case "position_y":
            update_position("y", message_value)

    # Controllo: se sta ruotando ritorna old state così non si crea coda ed esegue correttamente la rotaizone

    if not (controller._rotating or controller.rotation_done) and controller._target:
        client.publish("controls/target", "Finish")
        print("FINEEEEEEEEEEEEEE")
    else:
        control = controller.control_directions()
        # print("control:", control)
        if control != controller._old_perception:
            client.publish(f"controls/direction", control)
            print("control:", control)
            controller._old_perception = control


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")


if __name__ == "__main__":
    controller = (Controller("Brain",
                             # sostituire cross con turn left, turn right
                             possible_perceptions=[
                                 "go", "turn_left", "turn_right", "finish", "back"],
                             old_action="go"))

    client_mqtt = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.on_subscribe = on_subscribe
    client_mqtt.loop_forever()
