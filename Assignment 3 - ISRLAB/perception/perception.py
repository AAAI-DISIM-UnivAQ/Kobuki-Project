"""
    The perception module reads sensors from the sense_module
    and generates perceptions for the controller layer
"""

import paho.mqtt.client as mqtt

SENSE_HOST = "sense_module"
SENSE_PORT = 8080
MY_PORT = 8100
RETRY_DELAY = 250

MIN_DISTANCE = 0.5


class Perceptor:
    _old_perception: str
    _my_name: str
    _my_sensors: list
    _sensor_values: dict
    _my_possible_perceptions: list

    def __init__(self, name, sensors, perceptions):
        self._my_sensors = sensors
        self._my_perceptions = perceptions
        self._my_name = name

        self._sensor_values = {}
        for s in sensors:
            self._sensor_values[s] = 0
        self._old_perception = "front"

    # 0 --> sinistra, 4 --> centro, 7 --> destra.
    def is_free(self, values, pos):
        global i
        match pos:
            case "right":
                i = 7
            case "front":
                i = 4
            case "left":
                i = 0
        return values[f"ultrasonicSensor[{i}]"] == 0 or values[f"ultrasonicSensor[{i}]"] > MIN_DISTANCE

    def percept(self, values):
        if perceptor.is_free(values, "front"):
            if perceptor.is_free(values, "left") and perceptor.is_free(values, "right"):
                return "front"
            elif not (perceptor.is_free(values, "right")) and not (perceptor.is_free(values, "left")):
                return "front"
            elif not (perceptor.is_free(values, "left")) and perceptor.is_free(values, "right"):
                return "right"
            elif perceptor.is_free(values, "left") and not (perceptor.is_free(values, "right")):
                return "left"
        else:
            if perceptor.is_free(values, "left"):
                if perceptor.is_free(values, "right"):
                    return "right"
                else:
                    return "left"
        if perceptor.is_free(values, "left"):
            if perceptor.is_free(values, "front") and perceptor.is_free(values, "right"):
                return "left"
            elif not (perceptor.is_free(values, "front")) and not (perceptor.is_free(values, "right")):
                return "left"
            elif not (perceptor.is_free(values, "front")) and perceptor.is_free(values, "right"):
                return "right"
            elif not (perceptor.is_free(values, "right")) and perceptor.is_free(values, "front"):
                return "front"
        if perceptor.is_free(values, "right"):
            if perceptor.is_free(values, "front") or perceptor.is_free(values, "left"):
                return "right"
            elif not (perceptor.is_free(values, "front")) and not (perceptor.is_free(values, "left")):
                return "right"
            elif not (perceptor.is_free(values, "front")) and perceptor.is_free(values, "left"):
                return "left"
            elif not (perceptor.is_free(values, "left")) and perceptor.is_free(values, "front"):
                return "front"
        if values["ultrasonicSensor[0]"] != 0 and values["ultrasonicSensor[7]"] != 0:
            return "front"
        if (values["ultrasonicSensor[0]"] != 0 and values["ultrasonicSensor[7]"] != 0 and
                values["ultrasonicSensor[4]"] != 0):
            return "right"

    # docker-compose -f docker-compose.yml down --rmi all
    # docker compose up


def on_connect(client, userdata, flags, rc):
    client.subscribe("sense/+")
    return rc


def on_message(client, userdata, msg):
    global perceptor

    sensor_name = msg.topic.split("/")[1]
    sensor_value = float(msg.payload.decode())
    perceptor._sensor_values[sensor_name] = sensor_value

    new_perc = perceptor.percept(perceptor._sensor_values)

    if new_perc != perceptor._old_perception:
        client.publish("perception", new_perc)
        perceptor._old_perception = new_perc
    else:
        client.publish("perception", "front")


if __name__ == "__main__":
    perceptor = Perceptor("LittleBrain",
                          sensors=["ultrasonicSensor[0]", "ultrasonicSensor[4]", "ultrasonicSensor[7]"],
                          perceptions="freeSpace")

    client_pub = mqtt.Client("perception_publisher", reconnect_on_failure=True)
    client_pub.connect("mosquitto_module", 1883, 60)

    client_sub = mqtt.Client("perception_subscriber")
    client_sub.on_connect = on_connect
    client_sub.on_message = on_message
    client_sub.connect("mosquitto_module", 1883, 60)
    client_sub.loop_forever()
