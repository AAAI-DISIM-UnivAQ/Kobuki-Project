from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import array
import numpy as np
import cv2
from time import sleep


def detect_green_stripes(image):
    lower_green = np.array([45, 50, 50])
    upper_green = np.array([75, 255, 255])

    # Convertire l'immagine in formato HSV per facilitare la rilevazione del colore
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Creare una maschera per isolare le regioni verdi nell'immagine
    mask = cv2.inRange(hsv_image, lower_green, upper_green)

    # Applicare un'operazione morfologica per eliminare i piccoli dettagli
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Trovare i contorni delle strisce verdi nell'immagine
    contours, _ = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours


def calculate_desired_orientation(contours, image_width):
    if contours:
        # Trovare il contorno più grande
        largest_contour = max(contours, key=cv2.contourArea)
        # Calcolare il centroide del contorno più grande
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            # Calcolare l'orientamento desiderato rispetto al centro dell'immagine
            desired_orientation = (cx - image_width // 2) / (image_width // 2)
            return desired_orientation
    # Se non sono presenti contorni, mantenere lo stesso orientamento
    return 0


def move_robot(orientation):
    sim.setJointTargetVelocity(sim.getObject("./leftMotor"), 1)
    sim.setJointTargetVelocity(sim.getObject("./rightMotor"), 1)
    print("Orientamento desiderato:", orientation)


def capture_and_display_video(sim, handle, robot_handle, stop_distance):
    sim.startSimulation()

    try:
        while True:
            image, resolution = sim.getVisionSensorImg(handle)

            if image is not None and resolution is not None:
                img_array = array.array('B', image)
                img_np = np.array(img_array, dtype=np.uint8)
                img_np = img_np.reshape((resolution[1], resolution[0], 3))
                sleep(0.5)

                green_contours = detect_green_stripes(img_np)
                desired_orientation = calculate_desired_orientation(
                    green_contours, 256)
                move_robot(desired_orientation)

    except KeyboardInterrupt:
        pass
    finally:
        sim.stopSimulation()


client = RemoteAPIClient()
sim = client.getObject("sim")
vision_sensor_handle = sim.getObject("/Vision_sensor")
robot_handle = sim.getObject("/PioneerP3DX")
stop_distance = -150

capture_and_display_video(sim, vision_sensor_handle,
                          robot_handle, stop_distance)
