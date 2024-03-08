from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import matplotlib.pyplot as plt
import array
import numpy as np
import cv2
from time import sleep

def find_horizontal_and_vertical_lines(image, min_line_length=5, max_line_gap=10):
    # Estrai i canali di colore dall'immagine
    blue_channel, green_channel, red_channel = cv2.split(image)

    # Definisci un intervallo di valori per il verde (tweakare i valori a seconda del tuo caso)
    lower_green = np.array([0, 100, 0], dtype=np.uint8)
    upper_green = np.array([50, 255, 50], dtype=np.uint8)

    # Crea una maschera per i pixel verdi nell'intervallo definito
    green_mask = cv2.inRange(image, lower_green, upper_green)

    # Utilizza la trasformata di Hough probabilistica per individuare segmenti di linee
    lines = cv2.HoughLinesP(green_mask, 1, np.pi / 180, threshold=100, minLineLength=min_line_length, maxLineGap=max_line_gap)

    # Separazione delle linee orizzontali e verticali
    horizontal_lines = []
    vertical_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Calcola la pendenza della linea
            slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else float('inf')

            # Verifica se la linea è orizzontale o verticale
            if abs(slope) < 1:
                horizontal_lines.append(line)
            else:
                vertical_lines.append(line)

    # Disegna i segmenti di linee orizzontali sull'immagine di output
    output_horizontal = image.copy()
    num_horizontal_lines = len(horizontal_lines)
    for line in horizontal_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(output_horizontal, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # Disegna i segmenti di linee verticali sull'immagine di output
    output_vertical = image.copy()
    num_vertical_lines = len(vertical_lines)
    for line in vertical_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(output_vertical, (x1, y1), (x2, y2), (0, 0, 255), 2)

    return output_horizontal, output_vertical, num_horizontal_lines, num_vertical_lines


def capture_and_display_video(sim, handle, robot_handle, stop_distance):
    # Avvia la simulazione
    sim.startSimulation()

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

                # Trova linee orizzontali e verticali nell'immagine binaria
                output_horizontal, output_vertical, num_horizontal_lines, num_vertical_lines = find_horizontal_and_vertical_lines(img_np)

                # Crea una nuova figura con 3 nuovi subplot ad ogni iterazione
                fig, axs = plt.subplots(1, 3, figsize=(15, 5))

                # Visualizza l'immagine originale e i plot delle linee orizzontali e verticali
                axs[0].imshow(img_np, origin="lower")
                axs[0].set_title('Immagine Originale')
                axs[1].imshow(output_horizontal, cmap='gray', origin="lower")
                axs[1].set_title(f'Linee Orizzontali ({num_horizontal_lines})')
                axs[2].imshow(output_vertical, cmap='gray', origin="lower")
                axs[2].set_title(f'Linee Verticali ({num_vertical_lines})')

                # Aggiungi una pausa più lunga per evitare troppe richieste consecutive
                sleep(0.5)

                plt.pause(0.1)  # Aggiungi una piccola pausa per visualizzare l'immagine
                plt.close(fig)  # Chiudi la figura corrente

                # Ottieni la posizione del robot
                robot_position = sim.getObjectPosition(robot_handle, -1)

                # Controlla se la distanza è inferiore alla distanza di arresto desiderata
                # if green_stripe_distance > 0 and green_stripe_distance < stop_distance:
                #     print(f"Simulazione interrotta. Distanza dalla riga verde: {green_stripe_distance}")
                #     break

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
