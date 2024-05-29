from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import random
from collections import deque


def read_proximity_sensor(sensor):
    result, distance, detectedPoint, detected_object_handle, normalVector = sim.readProximitySensor(
        sensor)
    if result and detected_object_handle != -1:
        try:
            objectColor = sim.getObjectColor(
                detected_object_handle, 0, sim.colorcomponent_ambient_diffuse)
            return objectColor
        except Exception as e:
            print(
                f"Errore nell'ottenere il colore dell'oggetto con handle {detected_object_handle}: {e}")
            return None
    else:
        return [1.0, 1.0, 1.0]  # Assume white if no object is detected


def interpret_color(color):
    return color == [0.0, 0.0, 0.0]


client = RemoteAPIClient()
sim = client.getObject("sim")
robot_handle = sim.getObject("/PioneerP3DX")

# handle dei sensori di prossimità
front_sensor = sim.getObjectHandle("/Front_Proximity_sensor")
left_sensor = sim.getObjectHandle("/Left_Proximity_sensor")
right_sensor = sim.getObjectHandle("/Right_Proximity_sensor")

sim.startSimulation()

# handle dei motori delle ruote
left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

# Parameters for wheel speeds
base_speed = 2.0
turn_speed = 0.5
black = [0.0, 0.0, 0.0]

# State management
state = "FOLLOW_LINE"
history_length = 10  # Number of cycles to keep in history
sensor_history = deque(maxlen=history_length)

try:
    while True:
        front = read_proximity_sensor(front_sensor)
        left = read_proximity_sensor(left_sensor)
        right = read_proximity_sensor(right_sensor)

        on_line_left = interpret_color(left)
        on_line_right = interpret_color(right)
        on_line_front = interpret_color(front)

        sensor_state = (on_line_left, on_line_front, on_line_right)
        sensor_history.append(sensor_state)

        if state == "FOLLOW_LINE":
            # Only front sensor on line -> go straight
            if on_line_front and not on_line_right and not on_line_left:
                sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed)
                print("Go")
            # Front and left sensors on line -> adjust to the left
            elif on_line_front and on_line_left and not on_line_right:
                sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
                print("Adjust Left")
            # Front and right sensors on line -> adjust to the right
            elif on_line_front and on_line_right and not on_line_left:
                sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
                print("Adjust Right")
            # Use history to detect crossroad
            elif all(on_line_front and on_line_left and on_line_right for on_line_left, on_line_front, on_line_right in sensor_history):
                state = "CROSSROAD"
                decision_time = time.time()
            elif all(on_line_left and on_line_right and not on_line_front for on_line_left, on_line_front, on_line_right in sensor_history):
                state = "CROSSROAD"
                decision_time = time.time()
            # No sensor on line -> stop
            else:
                sim.setJointTargetVelocity(left_wheel_handle, 0)
                sim.setJointTargetVelocity(right_wheel_handle, 0)
                print("Stop")

        elif state == "CROSSROAD":
            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)
            time.sleep(1)
            rand = random.randint(1, 3)
            if rand == 1:
                sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
                print("Incrocio, vado a sinistra")
            elif rand == 2:
                sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed)
                print("Incrocio, vado dritto")
            else:
                sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
                print("Incrocio, vado a destra")
            state = "FOLLOW_LINE"
            time.sleep(5)

        # Optional: Add a small delay to avoid overwhelming the simulation
        time.sleep(0.05)

finally:
    sim.stopSimulation()
