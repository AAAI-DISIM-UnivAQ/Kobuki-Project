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

is_rotating = False

try:
    while True:

        sim.setJointTargetVelocity(left_wheel_handle, base_speed)
        sim.setJointTargetVelocity(right_wheel_handle, base_speed)
        time.sleep(1)

        front = read_proximity_sensor(front_sensor)
        left = read_proximity_sensor(left_sensor)
        right = read_proximity_sensor(right_sensor)

        on_line_left = interpret_color(left)
        on_line_right = interpret_color(right)
        on_line_front = interpret_color(front)

        # Solo sensore davanti --> vai dritto
        if on_line_front and not on_line_right and not on_line_left:
            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)

            print("Go")
        # Sensori davanti e sinistra --> scegli a caso tra davanti e sinistra
        elif on_line_front and on_line_left and not on_line_right:
            sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
            sim.setJointTargetVelocity(right_wheel_handle, turn_speed)

            print("Adjust Left")
        # Sensori davanti e destra --> scegli a caso tra davanti e destra
        elif on_line_front and on_line_right and not on_line_left:
            sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
            sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)

            print("Adjust Right")
        # Sensori davanti, sinistra e destra --> scegli a caso tra davanti, sinistra e destra
        elif on_line_front and on_line_left and on_line_right:

            rand = random.randint(1, 3)

            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)
            time.sleep(1)

            match rand:
                case 1:
                    sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, turn_speed)

                    print("Incrocio, vado a sinistra")
                case 2:
                    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, base_speed)

                    print("Incrocio, vado dritto")
                case 3:
                    sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)

                    print("Incrocio, vado a destra")

            time.sleep(5)
        # Sensori destra e sinistra --> scegli a caso tra destra e sinistra
        elif on_line_left and on_line_right and not on_line_front:
            rand = random.randint(1, 2)

            match rand:
                case 1:
                    sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, turn_speed)

                    print("Giro a sinistra")
                case 2:
                    sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)

                    print("Giro a destra")
            time.sleep(5)
        # Nessun sensore --> fermati
        elif not on_line_left and not on_line_front and not on_line_right:
            # Stop if no sensor detects the line
            sim.setJointTargetVelocity(left_wheel_handle, 0)
            sim.setJointTargetVelocity(right_wheel_handle, 0)
            print("Stop")

        # Optional: Add a small delay to avoid overwhelming the simulation
        time.sleep(0.05)

finally:
    sim.stopSimulation()
