from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject("sim")
sim.startSimulation()
handle = sim.getObjectHandle("visionSensor")
image, resolution, status = sim.getVisionSensorImage(handle)

if status == 0:
    print("Failed to retrieve vision sensor image.")
else:
    print("Image resolution:", resolution)
    print("Image data:", image)
