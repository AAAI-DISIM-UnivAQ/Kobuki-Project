import paho.mqtt.client as mqtt
import math
import time

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

    def __init__(self, name, possible_perceptions, old_action):
        """
        possible_perceptions: list of expected possible perception to handle
        """
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

    def control_directions(self):
        front = self._free_directions["front"]
        left = self._free_directions["left"]
        right = self._free_directions["right"]

        if front:
            if not left and not right:
                return "go"
            else:
                print("cross")
                # return "cross"
        else:
            if not left and not right:
                # return "back"
                return self.go_back()
            else:
                return "undetermined"  # Opzionale, per gestire altri casi se necessario

    def go_back(self):
        actual_angle = self._direction
        current_angle = self.normalize_angle(actual_angle)
        target_angle_back = 0
        if -0.3 < current_angle < 0.3:
            target_angle_back = math.pi
        elif -1.8 < current_angle < -1.2:
            target_angle_back = math.pi / 2
        elif current_angle < -2.8 or current_angle > 2.8:
            target_angle_back = 0
        elif 1.2 < current_angle < 1.8:
            target_angle_back = - math.pi / 2
        print("target_angle_back = ", target_angle_back)
        return self.set_robot_orientation(target_angle_back, "front")
        # self.go_straight()
        # self.set_speeds(0, 0)
        # time.sleep(1.0)

    def normalize_angle(self, angle):
        normalized_angle = angle % (2 * math.pi)
        if normalized_angle >= math.pi:
            normalized_angle -= 2 * math.pi
        return normalized_angle

    def set_robot_orientation(self, target_angle, dir):
        actual_angle = self._direction
        current_angle = self.normalize_angle(actual_angle)
        # print("current_angle = ", current_angle)
        diff = abs(abs(target_angle) - abs(current_angle))
        # print("diff", diff)
        # while diff > ANGLE_TOLERANCE:
        if diff > ANGLE_TOLERANCE:
            if dir == 'right' or dir == 'front':
                if diff > 0.8:
                    # self.set_speeds(TURN_SPEED, -TURN_SPEED)
                    return "turn_right"
                elif 0.3 < diff < 0.8:
                    # self.set_speeds(SLOW_TURN_SPEED, -SLOW_TURN_SPEED)
                    return "turn_right_slow"
                else:
                    # self.set_speeds(MORE_SLOW_TURN_SPEED, -MORE_SLOW_TURN_SPEED)
                    return "turn_right_more_slow"
            elif dir == 'left':
                if diff > 0.8:
                    # self.set_speeds(-TURN_SPEED, TURN_SPEED)
                    return "turn_left"
                elif 0.3 < diff < 0.8:
                    # self.set_speeds(-SLOW_TURN_SPEED, SLOW_TURN_SPEED)
                    return "turn_left_slow"
                else:
                    # self.set_speeds(-MORE_SLOW_TURN_SPEED, MORE_SLOW_TURN_SPEED)
                    return "turn_left_more_slow"
            # current_angle = self._sim_body.get_robot_orientation()
            # diff = abs(abs(target_angle) - abs(current_angle))


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

    print("name:", perception_name)

    match perception_name:
        case "front":
            print("Front", message_value)
            update_direction(perception_name, message_value)
        case "left":
            print("Left", message_value)
            update_direction(perception_name, message_value)
        case "right":
            print("Right", message_value)
            update_direction(perception_name, message_value)
        case "green":
            print("Green", message_value)
        case "orientation":
            print("Orientation", message_value)
            # client.publish(f"controls/{perception_name}", message_value)
            controller._direction = float(message_value)


    # Controllo: se sta ruotando ritorna old state così non si crea coda ed esegue correttamente la rotaizone

    control = controller.control_directions()
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
                             possible_perceptions=["go", "cross", "finish", "back"],  # sostituire cross con turn left, turn right
                             old_action="go"))

    client_mqtt = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.on_subscribe = on_subscribe
    client_mqtt.loop_forever()
