import paho.mqtt.client as mqtt
from SimulatedRobot import SimulatedPioneerBody
import time


class Body:
    _actuators: list
    _motions: list
    _sim_body: SimulatedPioneerBody
    _is_rotating: bool = False

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
        # print(f"Setting speeds: right = {right_speed}, left = {left_speed}")
        # assert self.exists_actuator(right_name)
        # assert self.exists_actuator(left_name)
        self.do_action("rightMotor", right_speed)
        self.do_action("leftMotor", left_speed)

    def can_move(self, direction):
        return direction in self._motions

    def rotate_180(self):
        print("DENTRO")
        # if self._is_rotating:
            # print("Already rotating, ignoring new rotation command.")
            # return

        # self._is_rotating = True
        print("Rotating 180 degrees")
        self.set_speeds(0, 0)  # Stop the robot
        right_motor_speed = -1.0  # Negative speed for backward motion
        left_motor_speed = 1.0  # Positive speed for forward motion
        rotation_time = 2.0  # Adjust this value according to your robot's specifications and simulator
        time.sleep(rotation_time)

        print(f"Starting rotation: right_motor_speed = {right_motor_speed}, left_motor_speed = {left_motor_speed}, rotation_time = {rotation_time}")
        self.set_speeds(right_motor_speed, left_motor_speed)
        time.sleep(rotation_time)  # Let the robot rotate for the specified time
        self.set_speeds(0, 0)  # Stop the robot
        print("Rotation complete")
        # self._is_rotating = False


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("controller/#")


def on_message(client, userdata, msg):
    # sim = SimulatedPioneerBody("Ritardato")
    print("---------")
    # values = {}
    name = msg.topic.split("/")[1]
    value = msg.payload.decode("utf-8")
    # values[name] = value
    print(f"{name}: {value}")

    print(my_robot._is_rotating)

    if my_robot._is_rotating:
        print("Robot is rotating, ignoring command.")
        # my_robot.set_speeds(0, 0)
        return

    match name:
        case "hor_distance":
            # if values[name] == "Stop":
            if value == "Stop":
                # my_robot._sim_body.stop()
                my_robot._is_rotating = True
                my_robot.rotate_180()
                # my_robot._is_rotating = True
                # my_robot.set_speeds(0, 0)
                # time.sleep(2)
                # my_robot._is_rotating = False
        # case "left_speed":
            # print(values[name])
        # case "right_speed":
            # print(values[name])
        case "correction":
            print("EXE Correction")
            # speeds = values[name].split(",")
            speeds = value.split(",")
            my_robot.set_speeds(speeds[0], speeds[1])


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
