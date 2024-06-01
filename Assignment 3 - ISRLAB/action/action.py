"""
    A Class which embeds the Robot body actuators
    in a docker container
    exposing its methods:
        - do_action
    through a HTTP service API
"""

from SimulatedRobot import SimulatedPioneerBody
from time import sleep
import paho.mqtt.client as mqtt

MOTION_WAIT = 0.5  # seconds to wait for the robot to rotate


# docker compose up

def on_connect(client, userdata, flags, rc):
    client.subscribe("controller")


def on_message(client, userdata, msg):
    command = msg.payload.decode()
    global my_body
    assert my_body.can_move(command)

    match command:
        case "forward":
            my_body.set_speeds(1, 1)
        case "right":
            my_body.set_speeds(-1, 1)
            sleep(MOTION_WAIT)
            my_body.set_speeds(0, 0)
        case "back_right":
            my_body.set_speeds(-0.5, -0.5)
            sleep(MOTION_WAIT)
            my_body.set_speeds(-1, 1)
            sleep(MOTION_WAIT)
        case "back_left":
            my_body.set_speeds(-0.5, -0.5)
            sleep(MOTION_WAIT)
            my_body.set_speeds(1, -1)
            sleep(MOTION_WAIT)
        case "left":
            my_body.set_speeds(1, -1)
            sleep(MOTION_WAIT)
            my_body.set_speeds(0, 0)
        case "stop":
            my_body.set_speeds(0, 0)
        case _:
            raise Exception(f"Unknown command {command}")


class Body:
    _actuators: list
    _motions: list
    _sim_body: SimulatedPioneerBody

    def __init__(self, actuators, motions):
        assert isinstance(actuators, list) and isinstance(motions, list)
        self._motions = motions
        self._actuators = actuators
        self._sim_body = SimulatedPioneerBody("Pioneer")
        self._sim_body.start()

    def exists_actuator(self, name):
        return name in self._actuators

    def do_action(self, actuator, value):
        self._sim_body.do_action(actuator, value)

    def set_speeds(self, right_speed, left_speed):
        self.do_action("rightMotor", right_speed)
        self.do_action("leftMotor", left_speed)

    def can_move(self, direction):
        return direction in self._motions

    def send_speeds(right_name, left_name, right_speed, left_speed):
        global my_body
        assert my_body.exists_actuator(right_name)
        assert my_body.exists_actuator(left_name)
        my_body.do_action(right_name, right_speed)
        my_body.do_action(left_name, left_speed)


if __name__ == "__main__":
    my_body = Body(["leftMotor", "rightMotor"],
                   ["forward", "back_right", "back_left",
                    "right", "left",
                    "stop"])

    client_sub = mqtt.Client("action_subscriber")
    client_sub.on_connect = on_connect
    client_sub.on_message = on_message
    client_sub.connect("mosquitto_module", 1883, 60)
    client_sub.loop_forever()
