from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import random
import math


def read_proximity_sensor(sensor):
    result, distance, detectedPoint, detected_object_handle, normalVector = sim.readProximitySensor(sensor)
    if result and detected_object_handle != -1:
        try:
            objectColor = sim.getObjectColor(detected_object_handle, 0, sim.colorcomponent_ambient_diffuse)
            return objectColor, distance
        except Exception as e:
            print(f"Errore nell'ottenere il colore dell'oggetto con handle {detected_object_handle}: {e}")
            return None, None
    else:
        return [1.0, 1.0, 1.0], None  # Assume white if no object is detected


def interpret_color(color):
    return color == [0.0, 0.0, 0.0]


def normalize_angle(angle):
    """
    Normalizza l'angolo nel range [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


client = RemoteAPIClient()
sim = client.getObject("sim")

# handle dei motori delle ruote
left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

# handle dei sensori di prossimità
front_sensor = sim.getObjectHandle("/Front_Proximity_sensor")
left_sensor = sim.getObjectHandle("/Left_Proximity_sensor")
right_sensor = sim.getObjectHandle("/Right_Proximity_sensor")

# Handle del robot
robot_handle = sim.getObjectHandle("/PioneerP3DX")  # Sostituisci con il percorso corretto del robot nel tuo ambiente

sim.startSimulation()

# Parameters for wheel speeds
base_speed = 0.5  # Velocità di base ridotta
turn_speed = 0.1  # Velocità di rotazione ridotta
intersection_threshold = 0.1  # Threshold distance to consider an intersection
intersection_delay = 1.0  # Tempo di attesa all'incrocio prima di girare

try:
    while True:
        # Lettura dei sensori
        front, front_distance = read_proximity_sensor(front_sensor)
        left, left_distance = read_proximity_sensor(left_sensor)
        right, right_distance = read_proximity_sensor(right_sensor)

        on_line_left = interpret_color(left)
        on_line_right = interpret_color(right)
        on_line_front = interpret_color(front)

        print("distances", left_distance, right_distance, front_distance)

        # Rileva un incrocio solo se i sensori laterali rilevano la linea entro un certo threshold
        if on_line_front and (((left_distance is not None and left_distance < intersection_threshold) and on_line_left) or (
                (right_distance is not None and right_distance < intersection_threshold) and on_line_right)):
            # Fermati
            sim.setJointTargetVelocity(left_wheel_handle, 0)
            sim.setJointTargetVelocity(right_wheel_handle, 0)
            print("Intersection detected, choosing direction...")

            # Scegli una direzione casuale
            options = []
            if (left_distance is not None and left_distance < intersection_threshold) and on_line_left:
                options.append("Left")
            if (right_distance is not None and right_distance < intersection_threshold) and on_line_right:
                options.append("Right")

            if options:  # Se ci sono opzioni disponibili
                direction = random.choice(options)

                # Aggiungi avanzamento aggiuntivo all'incrocio
                sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed)
                time.sleep(0.62)  # Avanza per 0.5 secondi
                sim.setJointTargetVelocity(left_wheel_handle, 0)
                sim.setJointTargetVelocity(right_wheel_handle, 0)
                time.sleep(intersection_delay)  # Tempo di attesa all'incrocio

                # Esegui l'azione corrispondente
                """
                if direction == "Right":
                    sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
                    print("Turning right")

                    # Calcola l'angolo di rotazione desiderato (90 gradi)
                    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]  # Angolo iniziale
                    target_angle = initial_angle + math.pi / 2  # Angolo di rotazione desiderato
                    while True:
                        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]  # Angolo corrente
                        if abs(current_angle - target_angle) < 0.05:  # Fermo quando l'angolo desiderato è raggiunto
                            break
                elif direction == "Left":
                    sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
                    print("Turning left")

                    # Calcola l'angolo di rotazione desiderato (90 gradi)
                    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]  # Angolo iniziale
                    target_angle = initial_angle - math.pi / 2  # Angolo di rotazione desiderato
                    while True:
                        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]  # Angolo corrente
                        if abs(current_angle - target_angle) < 0.05:  # Fermo quando l'angolo desiderato è raggiunto
                            break
                """
                if direction == "Right":
                    # Calcola l'angolo di rotazione desiderato (90 gradi)
                    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]  # Angolo iniziale
                    target_angle = normalize_angle(initial_angle - math.pi / 2)  # Angolo di rotazione desiderato

                    sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
                    print("Turning right")

                    while True:
                        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]  # Angolo corrente
                        current_angle = normalize_angle(current_angle)
                        if abs(normalize_angle(
                                current_angle - target_angle)) < 0.05:  # Fermo quando l'angolo desiderato è raggiunto
                            break

                elif direction == "Left":
                    # Calcola l'angolo di rotazione desiderato (90 gradi)
                    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]  # Angolo iniziale
                    target_angle = normalize_angle(initial_angle + math.pi / 2)  # Angolo di rotazione desiderato

                    sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
                    print("Turning left")

                    while True:
                        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]  # Angolo corrente
                        current_angle = normalize_angle(current_angle)
                        if abs(normalize_angle(
                                current_angle - target_angle)) < 0.05:  # Fermo quando l'angolo desiderato è raggiunto
                            break
            else:
                print("No options, going straight")

        # Altrimenti, vai dritto
        else:
            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)
            print("Going straight")

        # Optional: Add a small delay to avoid overwhelming the simulation
        time.sleep(0.05)

finally:
    sim.stopSimulation()
