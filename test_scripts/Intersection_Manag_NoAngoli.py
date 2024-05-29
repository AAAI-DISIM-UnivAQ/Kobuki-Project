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
base_speed = 0.5
turn_speed = 0.2
intersection_threshold = 0.1
intersection_delay = 2.0

choice_made = False

try:
    while True:
        # Lettura dei sensori
        front, front_distance = read_proximity_sensor(front_sensor)
        left, left_distance = read_proximity_sensor(left_sensor)
        right, right_distance = read_proximity_sensor(right_sensor)

        on_line_left = interpret_color(left)
        on_line_right = interpret_color(right)
        on_line_front = interpret_color(front)

        # Rileva un incrocio
        if on_line_front and (on_line_left or on_line_right):
            # Se non è già stata fatta una scelta
            if not choice_made:
                # Fermati
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

                    # Aggiorna il flag di scelta fatta
                    choice_made = True

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
                        print("Turning right")
                        # Aggiungi un ritardo per dare il tempo al robot di iniziare a girare
                        time.sleep(0.2)
                        while not interpret_color(read_proximity_sensor(front_sensor)[0]):
                            time.sleep(0.05)  # Continua a girare finché il sensore frontale non rileva la linea
                        sim.setJointTargetVelocity(left_wheel_handle, 0)
                        sim.setJointTargetVelocity(right_wheel_handle, 0)
                        print("Line detected, going straight")

                    elif direction == "Left":
                        sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
                        sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
                        print("Turning left")
                        # Aggiungi un ritardo per dare il tempo al robot di iniziare a girare
                        time.sleep(0.2)
                        while not interpret_color(read_proximity_sensor(front_sensor)[0]):
                            time.sleep(0.05)  # Continua a girare finché il sensore frontale non rileva la linea
                        sim.setJointTargetVelocity(left_wheel_handle, 0)
                        sim.setJointTargetVelocity(right_wheel_handle, 0)
                        print("Line detected, going straight")

                    elif direction == "Forward":
                        sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                        sim.setJointTargetVelocity(right_wheel_handle, base_speed)
                        print("Going forward")

                else:
                    print("No options, stopping")
                    sim.setJointTargetVelocity(left_wheel_handle, 0)
                    sim.setJointTargetVelocity(right_wheel_handle, 0)
            else:
                print("Choice already made, going straight")
        else:
            # Resetta il flag di scelta fatta quando i sensori non rilevano l'incrocio
            choice_made = False

            # Se nessuno dei tre sensori rileva la linea, fermati
            if not on_line_left and not on_line_right and not on_line_front:
                sim.setJointTargetVelocity(left_wheel_handle, 0)
                sim.setJointTargetVelocity(right_wheel_handle, 0)
                print("Line lost, stopping")
            else:
                # Altrimenti, vai dritto
                sim.setJointTargetVelocity(left_wheel_handle, base_speed)
                sim.setJointTargetVelocity(right_wheel_handle, base_speed)
                print("Going straight")

        # Optional: Aggiungi un piccolo ritardo per evitare sovraccarichi nella simulazione
        time.sleep(0.05)

finally:
    sim.stopSimulation()
