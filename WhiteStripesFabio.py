from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import array
import numpy as np
import cv2
from time import sleep


def count_white_lines(image, min_contour_length=0):
    contours, _ = cv2.findContours(
        image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    filtered_contours = [cnt for cnt in contours if cv2.arcLength(
        cnt, True) > min_contour_length]

    return len(filtered_contours)


def get_green_stripe_distance(image):

    lower_green = np.array([0, 100, 0], dtype=np.uint8)
    upper_green = np.array([50, 255, 50], dtype=np.uint8)

    green_mask = cv2.inRange(image, lower_green, upper_green)

    contours, _ = cv2.findContours(
        green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        average_distance = np.mean([cv2.pointPolygonTest(
            cnt, (image.shape[1] // 2, 0), True) for cnt in contours])
    else:
        average_distance = -1

    return average_distance


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

                robot_position = sim.getObjectPosition(robot_handle, -1)
                green_stripe_distance = get_green_stripe_distance(img_np)
                print(green_stripe_distance, stop_distance)

                if green_stripe_distance != -1 and green_stripe_distance >= stop_distance:
                    print(
                        f"Simulazione interrotta. Distanza dalla riga verde: {green_stripe_distance}")
                    break
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
stop_distance = -150

capture_and_display_video(sim, vision_sensor_handle,
                          robot_handle, stop_distance)
