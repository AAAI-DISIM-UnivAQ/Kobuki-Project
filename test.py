from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject("sim")
handle = sim.getObject("/Vision_sensor")

image, resolution = sim.getVisionSensorImg(handle)

print(image, resolution)
