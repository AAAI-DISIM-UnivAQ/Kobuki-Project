from SimulatedRobot import SimulatedPioneerBody
import paho.mqtt.client as mqtt


class Body:
    _sensor_array: list
    _d_sensors: dict
    _sim_body: SimulatedPioneerBody

    def __init__(self, sensors):
        assert isinstance(sensors, list)
        self._d_sensors = {}
        for sensor in sensors:
            self._d_sensors[sensor] = 0
        self._sensor_array = list(self._d_sensors.keys())
        self._sim_body = SimulatedPioneerBody("PioneerP3DX")
        self._sim_body.start()

    def sense(self, client):
        # global my_robot
        vision_values = self._sim_body.sense()
        for s in self._d_sensors:
            self._d_sensors[s] = vision_values[0]
        for name in self._sensor_array:
            client.publish(f"sense/{name}", str(self._d_sensors[name]))
            # print(f"Published ",str(self._d_sensors[name]))
            print("Published")


if __name__ == "__main__":
    my_robot = Body(["Vision_sensor"])

    client_mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)

    while True:
        my_robot.sense(client_mqtt)
