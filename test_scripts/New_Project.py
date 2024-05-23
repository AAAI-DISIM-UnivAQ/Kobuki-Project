from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import sys

client = RemoteAPIClient()
sim = client.getObject("sim")

# Ottieni i handle dei sensori di prossimità
front_sensor = sim.getObjectHandle("/Front_Proximity_sensor")
left_sensor = sim.getObjectHandle("/Left_Proximity_sensor")
right_sensor = sim.getObjectHandle("/Right_Proximity_sensor")

# Inizia la simulazione
client_id = sim.startSimulation()

# Ottieni i handle dei motori delle ruote
left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")


def read_proximity_sensor(sensor):
    result, _, _, detected_object_handle, _ = sim.readProximitySensor(sensor)
    if result and detected_object_handle:
        objectColor = sim.getObjectColor(
            detected_object_handle, 0, sim.colorcomponent_transparency)
        return objectColor
    else:
        print("Errore inaspettato (porcodio)")
        sys.exit()


try:
    # Variabile per tenere traccia dell'ultima direzione di movimento
    tracking_direction = None

    while True:
        # Leggi i valori dei sensori
        left_detection = read_proximity_sensor(left_sensor)
        right_detection = read_proximity_sensor(right_sensor)
        front_detection = read_proximity_sensor(front_sensor)

        # Controllo per seguire la linea
        if front_detection:
            # Se il sensore frontale rileva qualcosa, vai dritto
            sim.setJointTargetVelocity(left_wheel_handle, 1)
            sim.setJointTargetVelocity(right_wheel_handle, 1)
            tracking_direction = None  # Azzeriamo la direzione di tracking
        elif left_detection and not right_detection:
            # Se solo il sensore sinistro rileva la linea, gira leggermente a destra

            tracking_direction = 'right'
        elif right_detection and not left_detection:
            # Se solo il sensore destro rileva la linea, gira leggermente a sinistra

            tracking_direction = 'left'
        else:
            # Se nessun sensore rileva la linea, continua nella direzione precedente
            if tracking_direction == 'left':
                sim.setJointTargetVelocity(left_wheel_handle, 0.5)
                sim.setJointTargetVelocity(right_wheel_handle, 1)
            elif tracking_direction == 'right':
                sim.setJointTargetVelocity(left_wheel_handle, 1)
                sim.setJointTargetVelocity(right_wheel_handle, 0.5)
            else:
                # Se non abbiamo una direzione precedente, fermati
                sim.setJointTargetVelocity(left_wheel_handle, 0)
                sim.setJointTargetVelocity(right_wheel_handle, 0)

        # Aggiungi un piccolo ritardo per evitare di sovraccaricare la CPU
        time.sleep(0.1)

finally:
    # Ferma la simulazione in caso di interruzione
    sim.stopSimulation()
