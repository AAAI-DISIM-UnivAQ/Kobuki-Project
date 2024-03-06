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

# Ottieni l'immagine e la risoluzione
image, resolution = sim.getVisionSensorImg(handle)
sim.stopSimulation()
# Verifica la disponibilità dell'immagine
if image is not None and resolution is not None:
    # L'immagine è disponibile
    print("Immagine disponibile.")
    print("Risoluzione:", resolution)

    # Converti la stringa di byte in un array di byte
    img_array = array.array('B', image)

    # Converti l'array di byte in un array NumPy
    img_np = np.array(img_array, dtype=np.uint8)

    # Ridimensiona l'immagine in base alla risoluzione
    img_np = img_np.reshape((resolution[1], resolution[0], 3))

    # Mostra l'immagine utilizzando matplotlib
    plt.imshow(img_np)
    plt.show()

    # Puoi eseguire ulteriori operazioni qui
else:
    # L'immagine non è disponibile
    print("Immagine non disponibile.")
