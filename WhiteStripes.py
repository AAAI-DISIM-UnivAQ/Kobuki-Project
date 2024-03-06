from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import matplotlib.pyplot as plt
import array
import numpy as np
import cv2
from time import sleep

def get_green_stripe_distance(image, plot_binary_image=True):
    # Estrai i canali di colore dall'immagine
    blue_channel, green_channel, red_channel = cv2.split(image)

    # Definisci un intervallo di valori per il verde (tweakare i valori a seconda del tuo caso)
    lower_green = np.array([0, 100, 0], dtype=np.uint8)
    upper_green = np.array([50, 255, 50], dtype=np.uint8)

    # Crea una maschera per i pixel verdi nell'intervallo definito
    green_mask = cv2.inRange(image, lower_green, upper_green)

    # Trova i contorni nell'immagine binaria
    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Calcola la distanza media dalla linea verde utilizzando i contorni
    if contours:
        average_distance = np.mean([cv2.pointPolygonTest(cnt, (0, image.shape[0] // 2), True) for cnt in contours])
    else:
        average_distance = -1  # Imposta un valore impossibile se non ci sono contorni

    # Plot dell'immagine binaria se richiesto
    if plot_binary_image:
        plt.figure()
        plt.imshow(green_mask, cmap='gray', origin='lower')
        plt.title('Maschera verde')
        plt.show()

    return average_distance

def capture_and_display_video(sim, handle, robot_handle, stop_distance):
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
                #plt.imshow(img_np, origin="lower")
                plt.pause(0.1)  # Aggiungi una piccola pausa per visualizzare l'immagine

                # Aggiungi una pausa più lunga per evitare troppe richieste consecutive
                sleep(0.5)
                plt.clf()  # Pulisci la figura per l'immagine successiva

                # Ottieni la posizione del robot
                robot_position = sim.getObjectPosition(robot_handle, -1)

                # Calcola la distanza media dalla riga verde
                green_stripe_distance = get_green_stripe_distance(img_np)
                print("Distance from green stripe: ", green_stripe_distance)

                # Controlla se la distanza è inferiore alla distanza di arresto desiderata
                if green_stripe_distance > 0 and green_stripe_distance < stop_distance:
                    print(f"Simulazione interrotta. Distanza dalla riga verde: {green_stripe_distance}")
                    break

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
vision_sensor_handle = sim.getObject("/Vision_sensor")
robot_handle = sim.getObject("/PioneerP3DX")  # Sostituisci con il vero nome dell'handle del tuo robot
stop_distance = -50  # Sostituisci con la tua distanza di arresto desiderata

# Cattura e mostra il video
capture_and_display_video(sim, vision_sensor_handle, robot_handle, stop_distance)


