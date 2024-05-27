from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import sys
import time
import random
import math


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


def get_yaw(handle):
    # Ottieni l'orientamento del robot (angoli di Eulero)
    orientation = sim.getObjectOrientation(handle, -1)
    return orientation[2]  # Yaw (rotazione attorno all'asse Z)


def rotate_90_degrees(direction):
    initial_yaw = get_yaw(robot_handle)
    print("Angolo iniziale", initial_yaw)
    target_yaw = initial_yaw + (math.pi / 2 if direction == 'left' else -math.pi / 2)
    print("Angolo target", target_yaw)
    target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi  # Normalize the angle between -pi and pi
    print("Angolo normalizzato", target_yaw)

    while True:
        current_yaw = get_yaw(robot_handle)
        print("Angolo attuale:", current_yaw, "Angolo obiettivo", target_yaw)
        if direction == 'left':
            if abs(current_yaw - target_yaw) < 0.01:
                break
        elif direction == 'right':
            if abs(current_yaw - target_yaw) < 0.01:
                break

        sim.setJointTargetVelocity(left_wheel_handle, -turn_speed if direction == 'left' else turn_speed)
        sim.setJointTargetVelocity(right_wheel_handle, turn_speed if direction == 'left' else -turn_speed)
        # time.sleep(0.02)

    # Fermare il robot dopo la rotazione
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)
    print(f"Rotazione di 90 gradi completata verso {direction}")


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
            sim.setJointTargetVelocity(
                left_wheel_handle, -turn_speed)
            sim.setJointTargetVelocity(
                right_wheel_handle, turn_speed)

            print("Adjust Left")
        # Sensori davanti e destra --> scegli a caso tra davanti e destra
        elif on_line_front and on_line_right and not on_line_left:
            sim.setJointTargetVelocity(
                left_wheel_handle, turn_speed)
            sim.setJointTargetVelocity(
                right_wheel_handle, -turn_speed)

            print("Adjust Right")
        # Sensori davanti, sinistra e destra --> scegli a caso tra davanti, sinistra e destra
        elif on_line_front and on_line_left and on_line_right:

            rand = random.randint(1, 3)
            # rand = 3

            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)
            time.sleep(1)

            match rand:
                case 1:
                    # sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                    # sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
                    print("Incrocio, vado a sinistra")
                    rotate_90_degrees('left')
                case 2:
                    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, base_speed)

                    print("Incrocio, vado dritto")
                case 3:
                    # sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                    # sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
                    print("Incrocio, vado a destra")
                    rotate_90_degrees('right')

            # print("inizio sleep")
            # time.sleep(5)
            # print("fine sleep")
        # Sensori destra e sinistra --> scegli a caso tra destra e sinistra
        elif on_line_left and on_line_right and not on_line_front:
            rand = random.randint(1, 2)

            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)
            time.sleep(1)

            match rand:
                case 1:
                    # sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                    # sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
                    print("Giro a sinistra")
                    rotate_90_degrees('left')
                case 2:
                    # sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                    # sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
                    print("Giro a destra")
                    rotate_90_degrees('right')
            time.sleep(1)
        # Nessun sensore --> fermati
        elif not on_line_left and not on_line_front and not on_line_right:
            # Stop if no sensor detects the line
            sim.setJointTargetVelocity(left_wheel_handle, 0)
            sim.setJointTargetVelocity(right_wheel_handle, 0)
            print("Stop")

        # Optional: Add a small delay to avoid overwhelming the simulation
        # time.sleep(0.05)

finally:
    sim.stopSimulation()
