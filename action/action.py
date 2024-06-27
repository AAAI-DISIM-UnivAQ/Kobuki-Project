import paho.mqtt.client as mqtt
from SimulatedRobot import SimulatedPioneerBody
import threading
import math
import time

BASE_SPEED = 2.0
ANGLE_TOLERANCE = 0.02
TURN_SPEED = 0.3
SLOW_TURN_SPEED = 0.2
MORE_SLOW_TURN_SPEED = 0.1

class Body:
    _actuators: list
    # _motions: list
    _sim_body: SimulatedPioneerBody
    _go: bool
    # _orientation: float

    def __init__(self, actuators):
        assert isinstance(actuators, list)
        # self._motions = motions
        self._actuators = actuators
        self._sim_body = SimulatedPioneerBody("PioneerP3DX")
        self._sim_body.start()
        self._go = False
        # self._orientation = 0.0

    def exists_actuator(self, name):
        return name in self._actuators

    def do_action(self, actuator, value):
        print(f"Executing action on actuator {actuator} with value {value}")
        self._sim_body.do_action(actuator, value)

    def set_speeds(self, left_speed, right_speed):
        print(f"Setting speeds: right = {right_speed}, left = {left_speed}")
        self.do_action("leftMotor", left_speed)
        self.do_action("rightMotor", right_speed)


    def go_straight(self):
        self.set_speeds(BASE_SPEED,BASE_SPEED)
        self._go = True

    def go_back(self):
        actual_angle = self._sim_body.get_robot_orientation()
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
        self.set_robot_orientation(target_angle_back, "front")
        # self.go_straight()
        self.set_speeds(0, 0)
        time.sleep(1.0)

    def normalize_angle(self, angle):
        normalized_angle = angle % (2 * math.pi)
        if normalized_angle >= math.pi:
            normalized_angle -= 2 * math.pi
        return normalized_angle

    def set_robot_orientation(self, target_angle, dir):
        actual_angle = self._sim_body.get_robot_orientation()
        current_angle = self.normalize_angle(actual_angle)
        print("current_angle = ", current_angle)
        # if dir == "front":
            # diff = abs(target_angle - current_angle)
        # else:
            # diff = abs(abs(target_angle) - abs(current_angle))
        diff = abs(abs(target_angle) - abs(current_angle))
        #print("diff", diff)
        state = ""
        while diff > ANGLE_TOLERANCE:
            if dir == 'right' or dir == 'front':
                if diff > 0.8 and state != "turn":
                    self.set_speeds(TURN_SPEED, -TURN_SPEED)
                    state = "turn"
                elif 0.3 < diff < 0.8 and state == "turn":
                    self.set_speeds(SLOW_TURN_SPEED, -SLOW_TURN_SPEED)
                    state = "slow"
                elif diff < 0.3 and state == "slow":
                    self.set_speeds(MORE_SLOW_TURN_SPEED, -MORE_SLOW_TURN_SPEED)
                    state = "more"
            elif dir == 'left':
                if diff > 0.8:
                    self.set_speeds(-TURN_SPEED, TURN_SPEED)
                elif 0.3 < diff < 0.8:
                    self.set_speeds(-SLOW_TURN_SPEED, SLOW_TURN_SPEED)
                else:
                    self.set_speeds(-MORE_SLOW_TURN_SPEED, MORE_SLOW_TURN_SPEED)
            current_angle = self._sim_body.get_robot_orientation()
            # print('Current angle: ', current_angle)
            # if dir == "front":
                # diff = abs(target_angle - current_angle)
            # else:
                # diff = abs(abs(target_angle) - abs(current_angle))
            diff = abs(abs(target_angle) - abs(current_angle))
            # print("diff", diff)




def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("controller/#")


def on_message(client, userdata, msg):
    name = msg.topic.split("/")[1]
    value = msg.payload.decode("utf-8")
    # print(name, value)

    if name == "direction":
        if value == "go" and not my_robot._go:
            # my_robot.set_speeds(BASE_SPEED, BASE_SPEED)
            # my_robot._go = True
            my_robot.go_straight()
            print("Velocità base")
        elif value == "cross" and my_robot._go:
            # my_robot.set_speeds(0, 0)
            # my_robot._go = False
            print("Stop")
        elif value == "back" and my_robot._go:
            print("Back")
            client_mqtt.disconnect()
            my_robot._go = False
            my_robot.go_back()
            client_mqtt.reconnect()
    # elif name == "orientation":
        # my_robot._orientation = float(value)

    # client.publish("action", value, my_robot._go)


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        client.subscribe("controls/#")


if __name__ == "__main__":
    my_robot = Body(["leftMotor", "rightMotor"])

    client_mqtt = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.on_subscribe = on_subscribe
    client_mqtt.loop_forever()
