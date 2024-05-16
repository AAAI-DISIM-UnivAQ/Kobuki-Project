import paho.mqtt.client as mqtt
from SimulatedRobot import SimulatedPioneerBody


class Body:
    _actuators: list
    _motions: list
    _sim_body: SimulatedPioneerBody

    def __init__(self, actuators, motions):
        assert isinstance(actuators, list) and isinstance(motions, list)
        self._motions = motions
        self._actuators = actuators
        self._sim_body = SimulatedPioneerBody("PioneerP3DX")
        self._sim_body.start()

    def exists_actuator(self, name):
        return name in self._actuators

    def do_action(self, actuator, value):
        self._sim_body.do_action(actuator, value)

    def set_speeds(self, right_speed, left_speed):
        # assert self.exists_actuator(right_name)
        # assert self.exists_actuator(left_name)
        self.do_action("rightMotor", right_speed)
        self.do_action("leftMotor", left_speed)

    def can_move(self, direction):
        return direction in self._motions


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("controller/#")


def on_message(client, userdata, msg):
    # sim = SimulatedPioneerBody("Ritardato")

    values = {}
    name = msg.topic.split("/")[1]
    value = msg.payload.decode("utf-8")
    values[name] = value

    match name:
        case "hor_distance":
            if values[name] == "Stop":
                my_robot._sim_body.stop()
                print("STOP")
        case "left_speed":
            print(values[name])
        case "right_speed":
            print(values[name])


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        client.subscribe("controller/#")


if __name__ == "__main__":
    my_robot = Body(["leftMotor", "rightMotor"],
                    ["stop", "correct"])

    client_mqtt = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.on_subscribe = on_subscribe
    client_mqtt.loop_forever()
