"""
    A Class which embeds the Robot body sensors
    in a docker container
    exposing its methods:
        - get_sensor/<name>
        - get_sensors
    through a HTTP service API
"""

from SimulatedRobot import SimulatedPioneerBody
from urllib.parse import unquote
import paho.mqtt.client as mqtt


class Body:
    _sensor_array: list
    _d_sensors: dict
    _sim_body: SimulatedPioneerBody

    def __init__(self, sensors):
        assert isinstance(sensors, list)
        self._d_sensors = {}
        for s in sensors:
            self._d_sensors[s] = 0
        self._sensor_array = list(self._d_sensors.keys())
        self._sim_body = SimulatedPioneerBody("Pioneer")
        self._sim_body.start()

    def exists_sensor(self, name: str):
        assert isinstance(name, str)
        return name in self._sensor_array

    def sense(self, client):
        global my_body
        while True:
            front_sensor_values = self._sim_body.sense()
            for s in self._d_sensors:
                sid = s[-2:-1]
                self._d_sensors[s] = front_sensor_values[int(sid)]
            for name in my_body._sensor_array:
                client.publish(f"sense/{name}", self._d_sensors[name])

    def get_sensor_value(self, name):
        return self._d_sensors[name]


def get_sensor(name):
    global my_body
    sname = unquote(name)
    assert my_body.exists_sensor(sname)
    return str(my_body.get_sensor_value(sname))


def get_all_sensors():
    global my_body
    return str(my_body.sense())


if __name__ == "__main__":
    my_body = Body(["ultrasonicSensor[0]", "ultrasonicSensor[4]", "ultrasonicSensor[7]"])

    client_pub = mqtt.Client("sense_publisher", reconnect_on_failure=True)
    client_pub.connect("mosquitto_module", 1883, 60)
    my_body.sense(client_pub)
