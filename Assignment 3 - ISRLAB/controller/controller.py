"""
    The controller module reads perceptions from the perc_module
    and generates commands for the action layer
"""
import random
import paho.mqtt.client as mqtt
from time import sleep

PERC_HOST = "perc_module"
PERC_PORT = 8100
ACTION_HOST = "action_module"
ACTION_PORT = 8090


class Controller:
    _my_possible_perceptions: list
    _my_abilities: list
    _my_name: str
    _old_action: str
    _old_perception: str

    def __init__(self, name, possible_perceptions, abilities, old_action):
        """
        possible_perceptions: list of expected possible perception to handle
        """
        assert isinstance(name, str) and isinstance(possible_perceptions, list)
        self._my_name = name
        self._my_possible_perceptions = possible_perceptions
        self._my_abilities = abilities
        self._old_action = old_action
        self._old_perception = "front"


def on_connect(client, userdata, flags, rc):
    client.subscribe("perception")


def action_from_perception(perception):
    match perception:
        case "front":
            return "forward"
        case "right":
            return "right"
        case "left":
            return "left"
        case _:
            if random.random() > 0.5:
                return "back_right"
            else:
                return "back_left"


def on_message(client, userdata, msg):
    global controller

    perception = msg.payload.decode()
    client.publish("controller", action_from_perception(perception))


if __name__ == "__main__":
    client_pub = mqtt.Client("controller_publisher", reconnect_on_failure=True)
    client_pub.connect("mosquitto_module", 1883, 60)

    controller = (Controller("Brain",
                             possible_perceptions=["front", "left", "right", "everywhere", "back"],
                             abilities=["forward", "right", "left",
                                        "back_right", "back_left"],
                             old_action="forward"))

    client_sub = mqtt.Client("controller_subscriber")
    client_sub.on_connect = on_connect
    client_sub.on_message = on_message
    client_sub.connect("mosquitto_module", 1883, 60)
    client_sub.loop_forever()
