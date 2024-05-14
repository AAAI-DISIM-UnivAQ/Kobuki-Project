from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import matplotlib.pyplot as plt
import array
from time import sleep
import numpy as np

client = RemoteAPIClient()
sim = client.getObject("sim")
handle = sim.getObject("/Vision_sensor")
sim.startSimulation()
sleep(2)

image, resolution = sim.getVisionSensorImg(handle)
sim.stopSimulation()

if image is not None and resolution is not None:
    
    print("Immagine disponibile.")
    print("Risoluzione:", resolution)

    img_array = array.array('B', image)

    img_np = np.array(img_array, dtype=np.uint8)

    img_np = img_np.reshape((resolution[1], resolution[0], 3))

    plt.imshow(img_np, origin="lower")
    plt.show()

else:
    print("Immagine non disponibile.")
