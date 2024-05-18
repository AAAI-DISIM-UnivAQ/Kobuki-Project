from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import array
import numpy as np
import cv2
import math
from time import sleep

def detect_lane(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=100)

    left_line_x = []
    right_line_x = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1)
            if slope < 0:
                left_line_x.extend([x1, x2])
            elif slope > 0:
                right_line_x.extend([x1, x2])

    if left_line_x and right_line_x:
        left_line_x_avg = np.average(left_line_x)
        right_line_x_avg = np.average(right_line_x)
        center_lane_x = int((left_line_x_avg + right_line_x_avg) / 2)
    else:
        # Se non ci sono linee rilevate, assumiamo che il robot si trovi al centro della carreggiata
        center_lane_x = image.shape[1] // 2

    return center_lane_x

def get_horizontal_green_stripe_distance(image):
    lower_green = np.array([0, 100, 0], dtype=np.uint8)
    upper_green = np.array([50, 255, 50], dtype=np.uint8)

    green_mask = cv2.inRange(image, lower_green, upper_green)

    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        average_distance = np.mean([cv2.pointPolygonTest(cnt, (image.shape[1] // 2, 0), True) for cnt in contours])
    else:
        average_distance = -1

    return average_distance

def get_green_stripe_distance(image):
    center_lane_x = detect_lane(image)
    # Calcoliamo la distanza del robottino dal centro della carreggiata
    # La distanza sarà la differenza tra la posizione attuale del robottino e il centro della carreggiata
    distance_from_center = center_lane_x - image.shape[1] // 2

    return distance_from_center

def get_green_stripe_vertical_center(image):
    # Supponiamo che l'immagine sia in formato BGR
    # Convertiamo l'immagine in scala di grigi
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Applichiamo una sogliatura per ottenere solo i pixel verdi
    _, green_mask = cv2.threshold(gray_image, 70, 255, cv2.THRESH_BINARY)

    # Troviamo i contorni dei pixel verdi
    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Troviamo il contorno con l'area massima (la linea verde più lunga)
        largest_contour = max(contours, key=cv2.contourArea)

        # Calcoliamo il rettangolo del contorno
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Calcoliamo il centro verticale del rettangolo
        center_vertical = y + h / 2

        return center_vertical

    return None

def rotate_object_180_degrees_slowly(sim, robot_handle, left_wheel_handle, right_wheel_handle, rotation_speed=0.3):
    current_orientation = sim.getObjectOrientation(robot_handle, -1)
    target_orientation = current_orientation[2] + math.pi

    while True:
        current_orientation = sim.getObjectOrientation(robot_handle, -1)
        angle_diff = target_orientation - current_orientation[2]

        # Assicuriamoci che l'angolo sia tra -pi e pi per evitare rotazioni eccessive
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) < 0.01:  # Se la differenza è minore di un certo valore, interrompiamo la rotazione
            break

        # Impostiamo la velocità dei motori per ruotare il robot sul posto
        sim.setJointTargetVelocity(left_wheel_handle, -rotation_speed)
        sim.setJointTargetVelocity(right_wheel_handle, rotation_speed)

        sleep(0.01)  # Aggiungiamo un piccolo delay per dare il tempo al robot di aggiornare la posizione

    # Ferma i motori dopo aver completato la rotazione
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)

def capture_and_display_video(sim, handle, robot_handle, stop_distance, max_correction, image_width):
    sim.startSimulation()

    left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
    right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

    if left_wheel_handle is None or right_wheel_handle is None:
        print("Errore: impossibile ottenere i gestori delle ruote.")
        return

    try:
        while True:
            image, resolution = sim.getVisionSensorImg(handle)

            if image is not None and resolution is not None:
                img_array = array.array('B', image)
                img_np = np.array(img_array, dtype=np.uint8)
                img_np = img_np.reshape((resolution[1], resolution[0], 3))

                robot_position = sim.getObjectPosition(robot_handle, -1)
                green_stripe_distance = get_green_stripe_distance(img_np)
                horizontal_gsp = get_horizontal_green_stripe_distance(img_np)

                if horizontal_gsp != -1 and horizontal_gsp >= stop_distance:
                    print(f"Linea orizzontale rilevata. Distanza dalla riga verde: {horizontal_gsp}")
                    # Impostiamo velocità zero per fermare il robottino
                    sim.setJointTargetVelocity(left_wheel_handle, 0)
                    sim.setJointTargetVelocity(right_wheel_handle, 0)

                    # Ruotiamo di 180 gradi lentamente
                    rotate_object_180_degrees_slowly(sim, robot_handle, left_wheel_handle, right_wheel_handle)

                    # Dopo aver ruotato, riprendiamo il movimento in avanti
                    sim.setJointTargetVelocity(left_wheel_handle, 2)
                    sim.setJointTargetVelocity(right_wheel_handle, 2)

                else:
                    # Calcoliamo la correzione da applicare alla traiettoria
                    correction = green_stripe_distance

                    # Limitiamo la correzione massima per evitare movimenti eccessivamente bruschi
                    correction = max(-max_correction, min(max_correction, correction))

                    # Esempio di controllo per mantenere il robottino al centro della carreggiata
                    left_speed = 2 - correction / max_correction
                    right_speed = 2 + correction / max_correction

                    sim.setJointTargetVelocity(left_wheel_handle, left_speed)
                    sim.setJointTargetVelocity(right_wheel_handle, right_speed)

            else:
                print("Immagine non disponibile.")

    except KeyboardInterrupt:
        pass
    finally:
        sim.stopSimulation()

client = RemoteAPIClient()
sim = client.getObject("sim")
vision_sensor_handle = sim.getObject("/Vision_sensor")
robot_handle = sim.getObject("/PioneerP3DX")
# Impostiamo la massima correzione consentita per evitare movimenti bruschi
# Da regolare in base alle dimensioni dell'immagine e alla logica di controllo
max_correction = 50
stop_distance = -150
image_width = 640
capture_and_display_video(sim, vision_sensor_handle, robot_handle, stop_distance, max_correction, image_width)
