from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import random
import time
import math

DESIRED_DISTANCE_FROM_WALL = 0.5  # Distanza desiderata dalla parete centrale
CORRECTION_FACTOR = 0.1

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def turn_left():
    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]
    target_angle = normalize_angle(initial_angle + math.pi / 2)
    angle_difference = 0

    while angle_difference < math.pi / 2:
        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]
        current_angle = normalize_angle(current_angle)
        angle_difference = abs(normalize_angle(current_angle - initial_angle))

        sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
        sim.setJointTargetVelocity(right_wheel_handle, turn_speed)
        print("Turning left...")
        time.sleep(0.05)

    stop()


def turn_right():
    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]
    target_angle = normalize_angle(initial_angle - math.pi / 2)
    angle_difference = 0

    while angle_difference < math.pi / 2:
        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]
        current_angle = normalize_angle(current_angle)
        angle_difference = abs(normalize_angle(current_angle - initial_angle))

        sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
        sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
        print("Turning right...")
        time.sleep(0.05)

    stop()



def go_straight():

    left_dist = get_distance(left)
    right_dist = get_distance(right)
    dist_difference = right_dist - left_dist


    correction_speed = dist_difference * 0.1


    left_speed = base_speed + correction_speed
    right_speed = base_speed - correction_speed


    left_speed = min(max(left_speed, -base_speed), base_speed)
    right_speed = min(max(right_speed, -base_speed), base_speed)


    sim.setJointTargetVelocity(left_wheel_handle, left_speed)
    sim.setJointTargetVelocity(right_wheel_handle, right_speed)

def keep_distance_from_walls():
    left_dist = get_distance(left)
    right_dist = get_distance(right)

    # Calcola il punto medio tra le distanze ai muri
    mid_dist = (left_dist + right_dist) / 2

    # Calcola la differenza tra la distanza media e la distanza desiderata
    dist_difference = mid_dist - DESIRED_DISTANCE_FROM_WALL

    # Calcola le velocità correttive per ciascuna ruota
    correction_speed = dist_difference * CORRECTION_FACTOR


    left_correction_speed = base_speed + correction_speed
    right_correction_speed = base_speed - correction_speed

    # Assicura che le velocità correttive non superino le velocità massime
    left_correction_speed = min(max(left_correction_speed, -base_speed), base_speed)
    right_correction_speed = min(max(right_correction_speed, -base_speed), base_speed)

    #
    sim.setJointTargetVelocity(left_wheel_handle, left_correction_speed)
    sim.setJointTargetVelocity(right_wheel_handle, right_correction_speed)

def align_robot(target_angle):
    while True:
        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]
        current_angle = normalize_angle(current_angle)
        angle_difference = abs(normalize_angle(current_angle - target_angle))

        if angle_difference < 0.1:  # Soglia di allineamento
            break

        # Regola la velocità dei motori per allineare il robot
        sim.setJointTargetVelocity(left_wheel_handle, 0.1)
        sim.setJointTargetVelocity(right_wheel_handle, -0.1)
        print("Aligning robot...")
        time.sleep(0.05)

    # Dopo l'allineamento, fermiamo i motori
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)
    print("Robot aligned.")


def stop():
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)
    print("Stopping at intersection")


def get_distance(sensor):
    _, dist, _, _, _ = sim.readProximitySensor(sensor)
    return dist


def is_free(sensor):
    _, dist, _, _, _ = sim.readProximitySensor(sensor)
    return dist == 0 or dist > MIN_DISTANCE


def choose_direction(left_free, front_free, right_free):
    available_directions = []

    if left_free:
        available_directions.append("Left")
    if front_free:
        available_directions.append("Straight")
    if right_free:
        available_directions.append("Right")


    if not available_directions:
        return "TurnAround"

    # Seleziona casualmente una direzione tra quelle disponibili
    return random.choice(available_directions)

def turn_around():
    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]
    target_angle = normalize_angle(initial_angle + math.pi)

    while True:
        current_angle = sim.getObjectOrientation(robot_handle, -1)[2]
        current_angle = normalize_angle(current_angle)
        if current_angle >= target_angle:
            break

        sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
        sim.setJointTargetVelocity(right_wheel_handle, -turn_speed)
        print("Turning around...")
        time.sleep(0.05)
    stop()


direction_chosen = False

if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject("sim")

    robot_handle = sim.getObjectHandle("/PioneerP3DX")
    left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
    right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

    left = sim.getObjectHandle("/PioneerP3DX/ultrasonicSensor[0]")
    front = sim.getObjectHandle("/PioneerP3DX/ultrasonicSensor[4]")
    right = sim.getObjectHandle("/PioneerP3DX/ultrasonicSensor[7]")

    base_speed = 1.0
    turn_speed = 0.3
    MIN_DISTANCE = 0.4

    sim.startSimulation()
    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
    sim.setJointTargetVelocity(right_wheel_handle, base_speed)
    time.sleep(0.5)

    initial_angle = sim.getObjectOrientation(robot_handle, -1)[2]  # Definisci l'angolo iniziale qui
    try:
        while True:
            front_free = is_free(front)
            left_free = is_free(left)
            right_free = is_free(right)

            if not direction_chosen:  # Se il robot non ha ancora scelto una direzione all'incrocio
                if front_free and left_free and right_free:
                    time.sleep(1)
                    stop()
                    print("Intersection detected, stopping.")

                    direction = choose_direction(left_free, front_free, right_free)
                    if direction == "Left":
                        turn_left()
                    elif direction == "Right":
                        turn_right()
                    elif direction == "TurnAround":
                        turn_around()


                    direction_chosen = True


            else:
                if front_free:
                    go_straight()
                elif left_free:
                    turn_left()
                elif right_free:
                    turn_right()

                # Se tutti e tre i sensori rilevano un muro, gira di 180 gradi
                elif not (front_free or left_free or right_free):
                    turn_around()
                    direction_chosen = False

            time.sleep(0.05)

    finally:
        sim.stopSimulation()
