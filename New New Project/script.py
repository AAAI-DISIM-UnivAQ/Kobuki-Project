from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import random
import time


def turn_left():
    sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
    sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
    print("Turning left...")


def turn_right():
    sim.setJointTargetVelocity(left_wheel_handle, turn_speed)
    sim.setJointTargetVelocity(left_wheel_handle, -turn_speed)
    print("Turning right...")


def go_straight():
    sim.setJointTargetVelocity(left_wheel_handle, base_speed)
    sim.setJointTargetVelocity(right_wheel_handle, base_speed)
    print("Continuing straight")


def stop():
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)
    print("Stopping")


def turn_randomly(left, right, front):
    sim.setJointTargetVelocity(left_wheel_handle, 0)
    sim.setJointTargetVelocity(right_wheel_handle, 0)
    print("Intersection detected, choosing direction...")

    options = []
    if front:
        options.append("Front")
    if left:
        options.append("Left")
    if right:
        options.append("Right")

    if options:  # Se ci sono opzioni disponibili
        direction = random.choice(options)

        if direction == "Right":
            turn_right()
        elif direction == "Left":
            turn_left()
        elif direction == "Front":
            go_straight()
    else:
        print("Unexpected Error, stopping.")
        stop()


def is_free(sensor):
    _, perception, _, _, _ = sim.readProximitySensor(sensor)
    print(sensor, perception)
    return perception == 0 or perception > MIN_DISTANCE


if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject("sim")

    robot_handle = sim.getObjectHandle("/PioneerP3DX")
    left_wheel_handle = sim.getObjectHandle("/PioneerP3DX/leftMotor")
    right_wheel_handle = sim.getObjectHandle("/PioneerP3DX/rightMotor")

    left = sim.getObject("./ultrasonicSensor[0]")
    front = sim.getObject("./ultrasonicSensor[4]")
    right = sim.getObject("./ultrasonicSensor[7]")
    print(left, front, right)

    base_speed = 2.0
    turn_speed = 0.1
    MIN_DISTANCE = 0.5

    try:
        while True:
            if is_free(front):
                if not is_free(left) and not is_free(right):
                    go_straight()
                else:
                    turn_randomly(left=True, front=True, right=True)
            else:
                if not is_free(left) and not is_free(right):
                    stop()
                elif is_free(left) and not is_free(right):
                    turn_left()
                elif is_free(right) and not is_free(left):
                    turn_right()
                elif is_free(left) and is_free(right):
                    turn_randomly(left=True, front=False, right=True)

            time.sleep(0.05)

    finally:
        sim.stopSimulation()
