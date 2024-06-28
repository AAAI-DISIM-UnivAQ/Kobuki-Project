import paho.mqtt.client as mqtt
import array
import numpy as np
import cv2

MIN_DISTANCE = 0.5


class Perceptor:
    # _old_perception: str
    _my_name: str
    _my_sensors: list
    _sensor_values: dict
    # _my_possible_perceptions: list
    _perception: dict

    def __init__(self, name, sensors):
        self._my_sensors = sensors
        # self._my_perceptions = perceptions
        self._my_name = name
        self._perception = {}
        self._sensor_values = {}
        for s in sensors:
            self._sensor_values[s] = 0
        # self._old_perception = "front"

        # 0 --> sinistra, 4 --> centro, 7 --> destra.

    def is_free(sensor, dist):
        return float(dist) == 0 or float(dist) > MIN_DISTANCE

    def get_image_from_sensor(self, image, resolution):
        if image is not None and resolution is not None:
            img_array = array.array('B', image)
            img_np = np.array(img_array, dtype=np.uint8)
            img_np = img_np.reshape((resolution[1], resolution[0], 3))
            return img_np
        else:
            print(f"Failed to capture image from vision sensor.")
            return None

    def detect_green_object(self, image):
        # Convert the image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Define the range for green color in HSV
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        # Threshold the image to get only green colors
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        # Find contours in the masked image
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return len(contours) > 0

    def percept(self, values):
        for key, value in values.items():
            if key == "ultrasonicSensor[0]":
                # free = self.is_free(value)
                self._perception["left"] = self.is_free(value)
                print("Sinistra", self.is_free(value))
            elif key == "ultrasonicSensor[4]":
                self._perception["front"] = self.is_free(value)
                print("Centro", self.is_free(value))
            elif key == "ultrasonicSensor[7]":
                self._perception["right"] = self.is_free(value)
                print("Destra", self.is_free(value))
            elif key == "Vision_sensor":
                sensor_value = eval(value)
                image = sensor_value[0]
                resolution = sensor_value[1]
                img = self.get_image_from_sensor(image, resolution)
                if img is not None and self.detect_green_object(img):
                    print("Oggetto verde rilevato. Interrompendo la simulazione.")
                    self._perception["green"] = True
                else:
                    print("No green")
                    self._perception["green"] = False
            elif key == "orientation":
                print("Orientation", str(value))
                self._perception["orientation"] = value


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("sense/#")


# docker-compose -f docker-compose.yml down --rmi all


def on_message(client, userdata, msg):
    sensor_name = msg.topic.split("/")[1]
    message_value = msg.payload.decode("utf-8")
    print("Received")
    perceptor._sensor_values[sensor_name] = message_value

    perceptor.percept(perceptor._sensor_values)

    for key, value in perceptor._perception.items():
        client.publish(f"perception/{key}", str(value))


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")


if __name__ == "__main__":
    perceptor = Perceptor("LittleBrain", sensors=["Vision_sensor"])

    client_mqtt = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.on_subscribe = on_subscribe
    client_mqtt.loop_forever()
