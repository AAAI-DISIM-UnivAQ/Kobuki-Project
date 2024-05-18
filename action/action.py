import paho.mqtt.client as mqtt
from SimulatedRobot import SimulatedPioneerBody
import threading
import math


class Body:
    _actuators: list
    _motions: list
    _sim_body: SimulatedPioneerBody
    _is_rotating: bool = False
    _lock: threading.Lock

    def __init__(self, actuators, motions):
        assert isinstance(actuators, list) and isinstance(motions, list)
        self._motions = motions
        self._actuators = actuators
        self._sim_body = SimulatedPioneerBody("PioneerP3DX")
        self._sim_body.start()
        self._lock = threading.Lock()

    def exists_actuator(self, name):
        return name in self._actuators

    def do_action(self, actuator, value):
        print(f"Executing action on actuator {actuator} with value {value}")
        self._sim_body.do_action(actuator, value)

    def set_speeds(self, right_speed, left_speed):
        if not self._is_rotating:
            print(
                f"Setting speeds: right = {right_speed}, left = {left_speed}")
            self.do_action("rightMotor", right_speed)
            self.do_action("leftMotor", left_speed)

    def can_move(self, direction):
        return direction in self._motions

    def rotate_180_Giulia(self):
        rotation_speed = 0.3
        current_orientation = self._sim_body.getObjectOrientation(
            self._sim_body, -1)
        target_orientation = current_orientation[2] + math.pi

        while True:
            current_orientation = self._sim_body.getObjectOrientation(
                self._sim_body, -1)
            angle_diff = target_orientation - current_orientation[2]

            # Assicuriamoci che l'angolo sia tra -pi e pi per evitare rotazioni eccessive
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Se la differenza è minore di un certo valore, interrompiamo la rotazione
            if abs(angle_diff) < 0.01:
                break

            # Impostiamo la velocità dei motori per ruotare il robot sul posto
            self.set_speeds(-rotation_speed, -rotation_speed)

            # Aggiungiamo un piccolo delay per dare il tempo al robot di aggiornare la posizione
            # sleep(0.01)

        # Ferma i motori dopo aver completato la rotazione
        self.set_speeds(0, 0)

    def rotate_180(self):
        # def rotate():
        # with self._lock:
        while self._is_rotating:
            # self._is_rotating = True
            print("Rotating 180 degrees")
            self.set_speeds(0, 0)  # Stop the robot
            # Short pause to ensure the robot stops before rotating
            # time.sleep(0.5)

            right_motor_speed = -1.0  # Negative speed for backward motion
            left_motor_speed = -1.0  # Positive speed for forward motion
            # Adjust this value according to your robot's specifications and simulator
            rotation_time = 2.0

            print(
                f"Starting rotation: right_motor_speed={right_motor_speed}, left_motor_speed={left_motor_speed}, rotation_time={rotation_time}")
            self.set_speeds(right_motor_speed, left_motor_speed)
            # Let the robot rotate for the specified time
            # sleep(rotation_time)
            self.set_speeds(0, 0)  # Stop the robot
            print("Rotation complete")
            self._is_rotating = False

        # if not self._is_rotating:
        # threading.Thread(target=rotate).start()


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("controller/#")


def on_message(client, userdata, msg):
    name = msg.topic.split("/")[1]
    value = msg.payload.decode("utf-8")

    print("is_rotating:", my_robot._is_rotating)

    if my_robot._is_rotating:
        print("Robot is rotating, ignoring command.")
        return

    print("name:", name)

    match name:
        case "hor_distance":
            if value == "Stop":
                print("Stop")
                my_robot._is_rotating = True
                my_robot.rotate_180_Giulia()
                # time.sleep(0.5)
                my_robot._is_rotating = False
        case "correction":
            speeds = value.split(",")
            my_robot.set_speeds(speeds[0], speeds[1])
        case _:
            print("Errore inaspettato")
            my_robot._sim_body.stop()


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
