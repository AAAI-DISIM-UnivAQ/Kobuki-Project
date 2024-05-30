from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import random


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


client = RemoteAPIClient()
sim = client.getObject("sim")

# Handle del robot
robot_handle = sim.getObjectHandle("/PioneerP3DX")  # Sostituisci con il percorso corretto del robot nel tuo ambiente

# Handle dei motori delle ruote
left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

# Handle dei sensori di prossimità
front_sensor = sim.getObjectHandle("/Front_Proximity_sensor")
left_sensor = sim.getObjectHandle("/Left_Proximity_sensor")
right_sensor = sim.getObjectHandle("/Right_Proximity_sensor")

sim.startSimulation()

# Parameters for wheel speeds
base_speed = 2.0
turn_speed = 0.8
correction_speed = 0.5
intersection_delay = 1.0

try:
    while True:
        # Lettura dei sensori
        front, front_distance = read_proximity_sensor(front_sensor)
        left, left_distance = read_proximity_sensor(left_sensor)
        right, right_distance = read_proximity_sensor(right_sensor)

        # print("front", front, "left", left, "right")
        # print("front_distance", front_distance, "left_distance", left_distance, "right_distance", right_distance)

        on_line_left = interpret_color(left)
        on_line_right = interpret_color(right)
        on_line_front = interpret_color(front)

        # print("on_line_left", on_line_left, "on_line_right", on_line_right, "on_line_front", on_line_front)

        if on_line_front and (on_line_left or on_line_right):

            sim.setJointTargetVelocity(left_wheel_handle, 0)
            sim.setJointTargetVelocity(right_wheel_handle, 0)
            print("Intersection detected, choosing direction...")

            # Scegli una direzione casuale in base ai sensori
            options = []
            if on_line_left:
                options.append("Left")
            if on_line_right:
                options.append("Right")
            if on_line_front:
                options.append("Forward")

            if options:  # Se ci sono opzioni disponibili
                direction = random.choice(options)
                print(f"Chosen direction: {direction}")

                # Aggiungi avanzamento aggiuntivo all'incrocio
                sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed)
                time.sleep(0.5)  # Avanza per 0.5 secondi
                sim.setJointTargetVelocity(left_wheel_handle, 0)
                sim.setJointTargetVelocity(right_wheel_handle, 0)
                time.sleep(intersection_delay)  # Tempo di attesa all'incrocio

                # Esegui l'azione corrispondente
                if direction == "Right":
                    sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
                    time.sleep(0.5)
                    print("Turning right")
                    on_line_front = False
                    while not on_line_front:
                        sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
                        sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
                        front, front_distance = read_proximity_sensor(front_sensor)
                        on_line_front = interpret_color(front)
                        print("front", front, on_line_front, front_distance)

                    print("Rotation complete")
                    #time.sleep(0.25)
                    sim.setJointTargetVelocity(left_wheel_handle, 0)
                    sim.setJointTargetVelocity(right_wheel_handle, 0)

                elif direction == "Left":
                    sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
                    time.sleep(0.5)
                    print("Turning left")
                    on_line_front = False
                    while not on_line_front:
                        sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                        sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
                        front, front_distance = read_proximity_sensor(front_sensor)
                        on_line_front = interpret_color(front)
                        print("front", front, on_line_front, front_distance)
                    #time.sleep(0.15)
                    print("Rotation complete")
                    sim.setJointTargetVelocity(left_wheel_handle, 0)
                    sim.setJointTargetVelocity(right_wheel_handle, 0)

                elif direction == "Forward":
                    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, base_speed)
                    print("Going straight")

            else:
                print("No options, stopping")
                sim.setJointTargetVelocity(left_wheel_handle, 0)
                sim.setJointTargetVelocity(right_wheel_handle, 0)

        elif on_line_front and not (on_line_left or on_line_right):
            print("Go straight")
            sim.setJointTargetVelocity(left_wheel_handle, base_speed)
            sim.setJointTargetVelocity(right_wheel_handle, base_speed)

        """
        elif not on_line_front:
            if on_line_left:
                print("right correction")
                sim.setJointTargetVelocity(left_wheel_handle, base_speed - correction_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed + correction_speed)
            elif on_line_right:
                print("left correction")
                sim.setJointTargetVelocity(left_wheel_handle, base_speed + correction_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed - correction_speed)
            else:
                print("Stop")
                sim.setJointTargetVelocity(left_wheel_handle, 0)
                sim.setJointTargetVelocity(right_wheel_handle, 0)
        """

finally:
    sim.stopSimulation()
