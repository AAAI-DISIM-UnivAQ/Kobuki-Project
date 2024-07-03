import paho.mqtt.client as mqtt
import math
import time
import random

MAX_CORRECTION = 50
ANGLE_TOLERANCE = 0.02


class Controller:
    _my_possible_perceptions: list
    # _my_abilities: list
    _my_name: str
    _old_action: str
    _old_perception: str
    _free_directions: dict
    _direction: float
    _rotating: bool
    _target_angle: float
    _rotation_sense: str

    def __init__(self, name, possible_perceptions, old_action):
        assert isinstance(name, str) and isinstance(possible_perceptions, list)
        self._my_name = name
        self._my_possible_perceptions = possible_perceptions
        # self._my_abilities = abilities
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
            "right": False
        }

    def control_directions(self):
        global client_mqtt
        front = self._free_directions["front"]
        left = self._free_directions["left"]
        right = self._free_directions["right"]

        print("Rotating", self._rotating)

        # print("Rotazione", self._rotating)
        print("Front:", str(front), "Left:", str(left), "Right:", str(right))

        if self.update_direction["front"] and self.update_direction["left"] and self.update_direction["right"]:
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
                if self._old_perception == "cross":
                    print("SCELTA DELLA DIREZIONE")
                    self._rotating = True
                    return self.turn_randomly()
                else:
                    if front:
                        if not left and not right:
                            # strada dritta
                            return "go"
                        else:
                            # incrocio a 4
                            print("INCROCIO A 4")
                            client_mqtt.disconnect()
                            time.sleep(1.9)
                            client_mqtt.reconnect()
                            # self._rotating = True
                            # return self.turn_randomly()
                            return "cross"
                    else:
                        if left and right:
                            # incrocio a T
                            print("INCROCIO A T")
                            client_mqtt.disconnect()
                            time.sleep(1.9)
                            client_mqtt.reconnect()
                            self._rotating = True
                            return self.turn_randomly()
                        elif right:
                            # curva a destra
                            print("CURVA DX")
                            return self.turn_right()
                        elif left:
                            # curva a sinistra
                            print("CURVA SX")
                            return self.turn_left()
                        elif not left and not right:
                            # vicolo cieco
                            # return "back"
                            print("VICOLO CIECO")
                            self._rotating = True
                            return self.go_back()
                        else:
                            return "undetermined"  # Opzionale, per gestire altri casi se necessario
        else:
            return self.set_robot_orientation(self._target_angle, self._rotation_sense)

    def go_back(self):
        actual_angle = self._direction
        current_angle = self.normalize_angle(actual_angle)
        if -0.3 < current_angle < 0.3:
            self._target_angle = math.pi
        elif -1.8 < current_angle < -1.2:
            self._target_angle = math.pi / 2
        elif current_angle < -2.8 or current_angle > 2.8:
            self._target_angle = 0
        elif 1.2 < current_angle < 1.8:
            self._target_angle = - math.pi / 2
        self._rotation_sense = "front"
        return self.set_robot_orientation(self._target_angle, self._rotation_sense)

    def normalize_angle(self, angle):
        normalized_angle = angle % (2 * math.pi)
        if normalized_angle >= math.pi:
            normalized_angle -= 2 * math.pi
        return normalized_angle

    def set_robot_orientation(self, target_angle, dir):
        actual_angle = self._direction
        current_angle = self.normalize_angle(actual_angle)
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
            # return "rotation_done"
            self.rotation_done = True
            return "go"

    def turn_right(self):
        self.find_target_angle("right")
        self._rotation_sense = "right"
        return self.set_robot_orientation(self._target_angle, self._rotation_sense)

    def turn_left(self):
        self.find_target_angle("left")
        self._rotation_sense = "left"
        return self.set_robot_orientation(self._target_angle, self._rotation_sense)

    def turn_randomly(self):
        a = []
        for direction, is_free in self._free_directions.items():
            if is_free:
                a.append(direction)
        print("DIREZIONI DISPONIBILI", a)
        rand = random.choice(a)
        print("DIREZIONE SCELTA", rand)

        if rand == "front":
            self._rotating = False  # se non lo metto false dovrebbe superare incrocio tranquillamente
            return "go"
        elif rand == "right":
            return self.turn_right()
        elif rand == "left":
            return self.turn_left()

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


def update_direction(name, val):
    if val == "True":
        controller._free_directions[name] = True
    else:
        controller._free_directions[name] = False


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("perception/#")


def on_message(client, userdata, msg):
    perception_name = msg.topic.split("/")[1]
    message_value = msg.payload.decode("utf-8")

    # print("name:", perception_name)

    match perception_name:
        case "front":
            # print("Front", message_value)
            update_direction(perception_name, message_value)
        case "left":
            # print("Left", message_value)
            update_direction(perception_name, message_value)
        case "right":
            # print("Right", message_value)
            update_direction(perception_name, message_value)
        # case "green":
            # print("Green", message_value)
        case "orientation":
            # print("Orientation", message_value)
            controller._direction = float(message_value)

    # Controllo: se sta ruotando ritorna old state così non si crea coda ed esegue correttamente la rotaizone

    control = controller.control_directions()
    print("control:", control)
    if control != controller._old_perception:
        client.publish(f"controls/direction", control)
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

    print("Old perception", controller._old_perception)
    print("Free Directions", str(controller._free_directions))
    print("Direction", str(controller._direction))
    print("Rotating", str(controller._rotating))
    print("Target Angle", str(controller._target_angle))
    print("Rotation Sense", str(controller._rotation_sense))
    print("Rotation Done", str(controller.rotation_done))
    print("Waiting", str(controller.waiting_update_direction))
    print("Update Direction", str(controller.update_direction))

    client_mqtt = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.on_subscribe = on_subscribe

    # client_mqtt.publish(f"controls/direction", "go")

    client_mqtt.loop_forever()
