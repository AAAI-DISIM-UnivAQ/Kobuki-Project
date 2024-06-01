""""
    Encapsulates the communication with the robotic simulator
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any, List

# MY_SIM_HOST = "localhost"  # in PyCharm
MY_SIM_HOST = "host.docker.internal" # from the container

class SimulatedPioneerBody:
    _sim: Any
    _cSim_client: Any
    _my_sensors_values: List

    def __init__(self, name: str):
        self._my_name = name
        # zmqRemoteApi connection
        print("Connecting to simulator...")
        self._cSim_client = RemoteAPIClient(host=MY_SIM_HOST)
        self._sim = self._cSim_client.getObject('sim')
        print("Connected to SIM")
        self._my_sensors_values = []
        front_sensors = [
                   self._sim.getObject("./ultrasonicSensor[0]"),
                   self._sim.getObject("./ultrasonicSensor[1]"),
                   self._sim.getObject("./ultrasonicSensor[2]"),
                   self._sim.getObject("./ultrasonicSensor[3]"),
                   self._sim.getObject("./ultrasonicSensor[4]"),
                   self._sim.getObject("./ultrasonicSensor[5]"),
                   self._sim.getObject("./ultrasonicSensor[6]"),
                   self._sim.getObject("./ultrasonicSensor[7]")
        ]
        self._my_sensors_values.append(front_sensors)
        back_sensors = [
                self._sim.getObject("./ultrasonicSensor[8]"),
                self._sim.getObject("./ultrasonicSensor[9]"),
                self._sim.getObject("./ultrasonicSensor[10]"),
                self._sim.getObject("./ultrasonicSensor[11]"),
                self._sim.getObject("./ultrasonicSensor[12]"),
                self._sim.getObject("./ultrasonicSensor[13]"),
                self._sim.getObject("./ultrasonicSensor[14]"),
                self._sim.getObject("./ultrasonicSensor[15]")
        ]
        self._my_sensors_values.append(back_sensors)
        print("SIM objects referenced")


    def _read_sensors(self, i: int):
        # i = 0 : front sensors
        # i = 1 : back sensors
        assert 0 <= i <= 1, "incorrect sensor array"
        values = []
        for sens in self._my_sensors_values[i]:
            _, dis, _, _, _ = self._sim.readProximitySensor(sens)
            values.append(dis)
        return values

    def sense(self):
        """
        Read from (simulated-)hardware sensor devices and store into the internal array
        :return: True if all right, else hardware problem

        readProximitySensor:
        int result,float distance,list detectedPoint,int detectedObjectHandle,list detectedSurfaceNormalVector=sim.readProximitySensor(int sensorHandle)
        """
        try:
            front_values = self._read_sensors(0) # only front sensors
            return front_values
        except Exception as e:
            print(e)

    def start(self):
        self._sim.startSimulation()

    def stop(self):
        self._sim.stopSimulation()
