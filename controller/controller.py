import paho.mqtt.client as mqtt

MAX_CORRECTION = 50


class Controller:
    _my_possible_perceptions: list
    # _my_abilities: list
    _my_name: str
    _old_action: str
    _old_perception: str
    _free_directions: dict

    def __init__(self, name, possible_perceptions, old_action):
        """
        possible_perceptions: list of expected possible perception to handle
        """
        assert isinstance(name, str) and isinstance(possible_perceptions, list)
        self._my_name = name
        self._my_possible_perceptions = possible_perceptions
        # self._my_abilities = abilities
        self._old_action = old_action
        self._old_perception = "go"
        self._free_directions = {
            "front": True,
            "left": False,
            "right": False
        }

    def control_directions(self):
        front = self._free_directions["front"]
        left = self._free_directions["left"]
        right = self._free_directions["right"]

        if front:
            if not left and not right:
                return "go"
            else:
                return "cross"
        else:
            if not left and not right:
                return "back"
            else:
                return "undetermined"  # Opzionale, per gestire altri casi se necessario


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
            client.publish(f"controls/{perception_name}", message_value)

    control = controller.control_directions()
    client.publish(f"controls/direction", control)


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
