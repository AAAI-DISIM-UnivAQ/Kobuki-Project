from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import array
import cv2
import time

client = RemoteAPIClient()
sim = client.getObject("sim")
handle = sim.getObject("/Vision_sensor")

def detect_white_line():
    image, resolution = sim.getVisionSensorImg(handle)

    image = array.array("B", image)
    image = np.array(image, dtype=np.uint8)
    image = image.reshape(resolution[1], resolution[0], 3)  # 3 canali per l'immagine RGB
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Applica una sogliatura per ottenere un'immagine binaria
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    
    # Trova i contorni nell'immagine binaria
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Trova la lunghezza del contorno più lungo
    max_contour_length = 0
    for contour in contours:
        contour_length = cv2.arcLength(contour, True)
        if contour_length > max_contour_length:
            max_contour_length = contour_length
    
    # Imposta una soglia per la lunghezza del contorno per determinare se è una linea bianca
    line_threshold = 100  # Imposta un valore appropriato per la tua situazione
    
    # Se la lunghezza del contorno più lungo supera la soglia, ritorna True
    return max_contour_length > line_threshold

# Resto del codice rimane invariato

if __name__ == "__main__":
    sim.startSimulation()
    while True:
        white = detect_white_line()
        if white:
            print("fermati")
            sim.setJointTargetVelocity(sim.getObject("./leftMotor"), 0)
            sim.setJointTargetVelocity(sim.getObject("./rightMotor"), 0)
            sim.stopSimulation()
            break
        else:
            print("cammina")
            sim.setJointTargetVelocity(sim.getObject("./leftMotor"), 1)
            sim.setJointTargetVelocity(sim.getObject("./rightMotor"), 1)
        time.sleep(0.1)            

