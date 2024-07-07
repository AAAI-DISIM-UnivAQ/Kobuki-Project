from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any, List

MY_SIM_HOST = "host.docker.internal"


class SimulatedPioneerBody:
    _sim: Any
    _cSim_client: Any
    _my_actuators: dict
    _my_actuators_names: List
    _robot_handle: Any

    def __init__(self, name: str):
        self._my_name = name
        # zmqRemoteApi connection
        print("Connecting to simulator...")
        self._cSim_client = RemoteAPIClient(host=MY_SIM_HOST)
        self._sim = self._cSim_client.require('sim')
        print("Connected to SIM")
        self._robot_handle = self._sim.getObjectHandle("/PioneerP3DX") # per test
        self._my_actuators_names = ["leftMotor", "rightMotor"]
        # Get handles
        self._my_actuators = {}
        for act_name in self._my_actuators_names:
            self._my_actuators[act_name] = self._sim.getObject("./" + act_name)
        print("SIM objects referenced")

    def do_action(self, actuator_name, value):
        assert actuator_name in self._my_actuators_names
        actuator = self._my_actuators[actuator_name]
        # This is only for velocity values
        self._sim.setJointTargetVelocity(actuator, value)

    def start(self):
        self._sim.startSimulation()

    def stop(self):
        self._sim.stopSimulation()

    def get_robot_orientation(self):
        orientation = self._sim.getObjectOrientation(self._robot_handle, -1)
        return orientation[2]  # Restituisce l'angolo yaw

    def reset_actuators(self):
        for act_name in self._my_actuators_names:
            self._sim.setJointTargetVelocity(self._my_actuators[act_name], 0.0)
        print("Actuators reset to zero velocity")
