""""
    Encapsulates the communication with the robotic simulator
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any, List, Dict

MY_SIM_HOST = "host.docker.internal" # from the container
# MY_SIM_HOST ="localhost"


class SimulatedPioneerBody:
    _sim: Any
    _cSim_client: Any
    _my_actuators: dict
    _my_actuators_names: List

    def __init__(self, name: str):
        self._my_name = name
        # zmqRemoteApi connection
        print("Connecting to simulator...")
        self._cSim_client = RemoteAPIClient(host=MY_SIM_HOST)
        self._sim = self._cSim_client.getObject('sim')
        print("Connected to SIM")
        self._my_actuators_names = ["leftMotor", "rightMotor"]
        # Get handles
        self._my_actuators = {}
        for act_name in self._my_actuators_names:
            self._my_actuators[act_name] = self._sim.getObject("./" + act_name)
        print("SIM objects referenced")

    def do_action(self, actuator_name, value):
       """
       :param actuator: simulator name reference
       :param value: scalar value to impose to the simulated actuator (speed, angle)
       :return:
       """
       assert actuator_name in self._my_actuators_names
       actuator = self._my_actuators[actuator_name]
       # This is only for velocity values
       self._sim.setJointTargetVelocity(actuator, value)


    def start(self):
        self._sim.startSimulation()

    def stop(self):
        self._sim.stopSimulation()
