from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import matplotlib.pyplot as plt
import array
import numpy as np
from time import sleep

def capture_and_display_video(sim, handle):
    # Avvia la simulazione
    sim.startSimulation()
    # sleep(2)  # Se necessario, puoi aggiungere una pausa iniziale

    try:
        while True:
            # Ottieni l'immagine e la risoluzione
            image, resolution = sim.getVisionSensorImg(handle)

            # Verifica la disponibilità dell'immagine
            if image is not None and resolution is not None:
                # Converti la stringa di byte in un array di byte
                img_array = array.array('B', image)

                # Converti l'array di byte in un array NumPy
                img_np = np.array(img_array, dtype=np.uint8)

                # Ridimensiona l'immagine in base alla risoluzione
                img_np = img_np.reshape((resolution[1], resolution[0], 3))

                # Mostra l'immagine utilizzando matplotlib
                plt.imshow(img_np, origin="lower")
                plt.pause(0.1)  # Aggiungi una piccola pausa per visualizzare l'immagine

                # Aggiungi una pausa più lunga per evitare troppe richieste consecutive
                sleep(0.1)
                plt.clf()  # Pulisci la figura per l'immagine successiva
            else:
                # L'immagine non è disponibile
                print("Immagine non disponibile.")
    except KeyboardInterrupt:
        # Interrompi la simulazione alla pressione di Ctrl+C
        pass
    finally:
        # Ferma la simulazione al termine
        sim.stopSimulation()

# Crea un'istanza del client
client = RemoteAPIClient()
sim = client.getObject("sim")
handle = sim.getObject("/Vision_sensor")

# Cattura e mostra il video
capture_and_display_video(sim, handle)
