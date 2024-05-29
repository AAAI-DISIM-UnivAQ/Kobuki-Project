from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import random


def read_proximity_sensor(sensor):
    result, distance, detectedPoint, detected_object_handle, normalVector = sim.readProximitySensor(
        sensor)
    if result and detected_object_handle != -1:
        try:
            objectColor = sim.getObjectColor(
                detected_object_handle, 0, sim.colorcomponent_ambient_diffuse)
            return objectColor, distance
        except Exception as e:
            print(
                f"Error obtaining color of the object with handle {detected_object_handle}: {e}")
            return None, None
    else:
        return [1.0, 1.0, 1.0], None  # Assume white if no object is detected


def interpret_color(color):
    if color == [0.0, 0.0, 0.0]:
        return "Black"
    elif color == [1.0, 1.0, 1.0]:
        return "White"
    elif color == [0.5, 0.5, 0.5]:
        return "Gray"
    else:
        return None


client = RemoteAPIClient()
sim = client.getObject("sim")

# Robot and motor handles
robot_handle = sim.getObjectHandle("/PioneerP3DX")
left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

# Proximity sensor handles
front_sensor = sim.getObjectHandle("/Front_Proximity_sensor")
left_sensor = sim.getObjectHandle("/Left_Proximity_sensor")
right_sensor = sim.getObjectHandle("/Right_Proximity_sensor")

sim.startSimulation()

# Parameters for wheel speeds
base_speed = 0.5
turn_speed = 0.2

try:
    while True:
        # Read sensors
        front, front_distance = read_proximity_sensor(front_sensor)
        left, left_distance = read_proximity_sensor(left_sensor)
        right, right_distance = read_proximity_sensor(right_sensor)

        on_line_left = interpret_color(left)
        on_line_right = interpret_color(right)
        on_line_front = interpret_color(front)

        if on_line_front == "Black" and on_line_left == "White" and on_line_right == "White":
            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)

        elif on_line_front == "Black" and on_line_left == "Black" and on_line_right == "Black":
            # Random decision for intersections
            rand = random.randint(1, 3)
            rand = 1
            print("Incrocio a 3")

            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)
            time.sleep(0.5)

            if rand == 1:  # Turn left
                sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, turn_speed)

                print("Turning left")
                time.sleep(0.2)
                while not (interpret_color(read_proximity_sensor(front_sensor)[0]) == "Black") and not (interpret_color(read_proximity_sensor(left_sensor)[0]) == "White") and not (interpret_color(read_proximity_sensor(right_sensor)[0]) == "White"):
                    print("Turning...")
                    time.sleep(0.05)

            elif rand == 2:  # Go straight
                sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed)

                print("Going straight")
            elif rand == 3:  # Turn right
                sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)

                print("Turning right")
                while not (interpret_color(read_proximity_sensor(front_sensor)[0]) == "Black"):
                    time.sleep(0.05)

        elif on_line_front == "Black" and on_line_left == "Black" and on_line_right == "White":
            rand = random.randint(1, 2)
            print("Incrocio a 2")

            sim.setJointTargetVelocity(left_wheel_handle, 0)
            sim.setJointTargetVelocity(right_wheel_handle, 0)
            time.sleep(1)

            if rand == 1:
                sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed)

                print("Going straight")
            elif rand == 2:
                sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, turn_speed)

                print("Turning left")
                while not (interpret_color(read_proximity_sensor(front_sensor)[0]) == "Black") or not (interpret_color(read_proximity_sensor(left_sensor)[0]) == "White") or not (interpret_color(read_proximity_sensor(right_sensor)[0]) == "White"):
                    time.sleep(0.05)

        elif on_line_front == "Black" and on_line_left == "White" and on_line_right == "Black":
            rand = random.randint(1, 2)
            print("Incrocio a 2")

            sim.setJointTargetVelocity(left_wheel_handle, 0)
            sim.setJointTargetVelocity(right_wheel_handle, 0)
            time.sleep(1)

            if rand == 1:
                sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed)

                print("Going straight")
            elif rand == 2:
                sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)

                print("Turning right")
                while not (interpret_color(read_proximity_sensor(front_sensor)[0]) == "Black") or not (interpret_color(read_proximity_sensor(left_sensor)[0]) == "White") or not (interpret_color(read_proximity_sensor(right_sensor)[0]) == "White"):
                    time.sleep(0.05)

        elif on_line_front == "White" and on_line_left == "Black" and on_line_right == "Black":
            rand = random.randint(1, 2)
            print("Incrocio a 2")

            sim.setJointTargetVelocity(left_wheel_handle, 0)
            sim.setJointTargetVelocity(right_wheel_handle, 0)
            time.sleep(1)

            if rand == 1:
                sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, turn_speed)

                print("Turning left")
                while not (interpret_color(read_proximity_sensor(front_sensor)[0]) == "Black") or not (interpret_color(read_proximity_sensor(left_sensor)[0]) == "White") or not (interpret_color(read_proximity_sensor(right_sensor)[0]) == "White"):
                    time.sleep(0.05)
            elif rand == 2:
                sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)

                print("Turning right")
                while not (interpret_color(read_proximity_sensor(front_sensor)[0]) == "Black") or not (interpret_color(read_proximity_sensor(left_sensor)[0]) == "White") or not (interpret_color(read_proximity_sensor(right_sensor)[0]) == "White"):
                    time.sleep(0.05)

        elif on_line_left is None and on_line_right is not None:
            sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
            sim.setJointTargetVelocity(right_wheel_handle, turn_speed)

            print("Adjust left")
        elif on_line_right is None and on_line_left is not None:
            sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
            sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)

            print("Adjust right")
        elif on_line_left is None and on_line_right is None:
            print("Diocane")

        else:
            sim.setJointTargetVelocity(left_wheel_handle, 0)
            sim.setJointTargetVelocity(right_wheel_handle, 0)

            print("Stopping")

        time.sleep(0.1)

finally:
    sim.stopSimulation()
