import paho.mqtt.client as mqtt
from SimulatedRobot import SimulatedPioneerBody


class Perceptor:
    _sensor_values: dict
    _sim_body: SimulatedPioneerBody

    def __init__(self):
        pass


def on_connect(client, userdata, flags, rc):
    client.subscribe("sense/Vision_sensor")
    return rc

# docker-compose -f docker-compose.yml down --rmi all


def on_message(client, userdata, msg):
    global perceptor

    sensor_value = msg.payload.decode()

    coded_image = sensor_value[:-2]
    resolution = sensor_value[-2:]


if __name__ == "__main__":

    percepetor = Perceptor()

    client_pub = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_pub.connect("perception", 1883)

    client_sub = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_sub.on_connect = on_connect
    client_sub.on_message = on_message
    client_sub.connect("sense", 1883)
    client_sub.loop_forever()
