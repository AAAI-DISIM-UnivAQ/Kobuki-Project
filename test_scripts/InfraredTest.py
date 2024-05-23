from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import sys
import time


def read_proximity_sensor(sensor):
    result, distance, detectedPoint, detected_object_handle, normalVector = sim.readProximitySensor(sensor)
    if result and detected_object_handle != -1:
        try:
            objectColor = sim.getObjectColor(detected_object_handle, 0, sim.colorcomponent_ambient_diffuse)
            return objectColor
        except Exception as e:
            print(f"Errore nell'ottenere il colore dell'oggetto con handle {detected_object_handle}: {e}")
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

try:
    while True:
        front = read_proximity_sensor(front_sensor)
        left = read_proximity_sensor(left_sensor)
        right = read_proximity_sensor(right_sensor)

        # print("Front Sensor", front)
        # print("Left Sensor", left)
        # print("Right Sensor", right)

        on_line_left = interpret_color(left)
        on_line_right = interpret_color(right)
        on_line_front = interpret_color(front)

        if on_line_front and not on_line_right and not on_line_left:
            # Move forward if the front sensor detects the line
            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)
            print("GO")
        elif on_line_front and on_line_left and not on_line_right:
            # Turn left if the left sensor detects the line but not the right sensor
            sim.setJointTargetVelocity(left_wheel_handle, base_speed - turn_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed + turn_speed)
            print("Adjust Left")
        elif on_line_front and on_line_right and not on_line_left:
            # Turn right if the right sensor detects the line but not the left sensor
            sim.setJointTargetVelocity(left_wheel_handle, base_speed + turn_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed - turn_speed)
            print("Adjust Right")
        elif on_line_front and on_line_left and on_line_right:
            print("Incrocio")
        else:
            # Stop if no sensor detects the line
            sim.setJointTargetVelocity(left_wheel_handle, 0)
            sim.setJointTargetVelocity(right_wheel_handle, 0)
            print("Stop")

        # Optional: Add a small delay to avoid overwhelming the simulation
        time.sleep(0.05)

finally:
    sim.stopSimulation()

