"""
    A Class which embeds the Agent body
    in a docker container
    exposing its methods:
        - read_sensors
        - do_action
    trhough a HTTP service API
"""

from bottle import route, run


class Body:

    _sensor_array: list
    _d_sensors: dict
    _action_names: list

    def __init__(self, sensors):
        assert isinstance(sensors, list)
        self._d_sensors = {}
        for s in sensors:
            self._d_sensors[s] = 0
        self._sensor_array = list(self._d_sensors.keys())
        self._action_names = ["speed_right", "speed_left"]

    def exists_sensor(self, name: str):
        assert isinstance(name, str)
        return name in self._sensor_array

    def get_sensor_value(self, name):
        return self._d_sensors[name]

    def speed_right(self, speed: float):
        """
        :param speed: right wheel speed in m/s
        :return: None
        """
        pass

    def speed_left(self, speed: float):
        """
        :param speed: left wheel speed in m/s
        :return: None
        """
        pass


@route("/get_sensor/<name>")
def get_sensor(name):
    global my_body
    assert my_body.exists_sensor(name)
    return str(my_body.get_sensor_value(name))


if __name__ == "__main__":
    print("starting")
    my_body = Body(["whisker", "Distance"])
    run(host="0.0.0.0", debug=True, port=8080)
