import paho.mqtt.client as mqtt
import array
import numpy as np
import cv2

STOP_DISTANCE = -150
MAX_CORRECTION = 50


class Perceptor:
    # _old_perception: str
    _my_name: str
    _my_sensors: list

    # _sensor_values: dict

    def __init__(self, name, sensors, perceptions):
        assert isinstance(sensors, list) and isinstance(perceptions, list)
        self._my_sensors = sensors
        self._my_perceptions = perceptions
        self._my_name = name
        # self._sensor_values = {}
        # for s in sensors:
        # self._sensor_values[s] = 0
        # self._old_perception = "free"

    def percept(self, img, client):
        road_center_distance = self.get_distance_from_center(img)
        horizontal_gps = self.get_horiziontal_green_stripes_distance(img)
        stop = False
        correction = 0

        if horizontal_gps != -1 and horizontal_gps >= STOP_DISTANCE:
            print("Horizontal STOP DISTANCE reached")
            stop = True
        client.publish(f"perception/hor_distance", stop)

        trajectory_correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, road_center_distance))
        if trajectory_correction != 0:
            print("Trajectory correction")
            correction = trajectory_correction
        client.publish(f"perception/correction", correction)

        # return stop, correction

    def get_distance_from_center(self, img):
        center_lane_x = self.detect_lane(img)

        # Calcoliamo la distanza del robottino dal centro della carreggiata
        # La distanza sarà la differenza tra la posizione attuale del robottino e il centro della carreggiata
        distance_from_center = center_lane_x - img.shape[1] // 2

        return distance_from_center

    def detect_lane(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                                threshold=50, minLineLength=50, maxLineGap=100)

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
            center_lane_x = img.shape[1] // 2

        return center_lane_x

    def get_horiziontal_green_stripes_distance(self, img):

        lower_green = np.array([0, 100, 0], dtype=np.uint8)
        upper_green = np.array([50, 255, 50], dtype=np.uint8)

        green_mask = cv2.inRange(img, lower_green, upper_green)

        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            average_distance = np.mean([cv2.pointPolygonTest(
                cnt, (img.shape[1] // 2, 0), True) for cnt in contours])
        else:
            average_distance = -1

        return average_distance


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("sense/#")


# docker-compose -f docker-compose.yml down --rmi all


def on_message(client, userdata, msg):
    sensor_name = msg.topic.split("/")[1]
    message_value = msg.payload.decode("utf-8")
    print("Recived")
    sensor_value = eval(message_value)

    image = sensor_value[0]
    resolution = sensor_value[1]

    if image is not None and resolution is not None:
        img_array = array.array('B', image)
        img_np = np.array(img_array, dtype=np.uint8)
        img_np = img_np.reshape((resolution[1], resolution[0], 3))

        percepetor.percept(img_np, client_mqtt)
        # new_perc = percepetor.percept(img_np)
        # client_mqtt.publish(f"perception/", str(new_perc))

    else:
        print("Image not available.")


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")


if __name__ == "__main__":
    percepetor = Perceptor("LittleBrain",
                           sensors=["Vision_sensor"],
                           perceptions=["free"])

    client_mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.on_subscribe = on_subscribe
    client_mqtt.loop_forever()
