import paho.mqtt.client as mqtt
from SimulatedRobot import SimulatedPioneerBody


class Action:
    pass


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("controller/+")


def on_message(client, userdata, msg):
    sim = SimulatedPioneerBody("Ritardato")

    values = {}
    name = msg.topic.split("/")[1]
    value = msg.payload.decode("utf-8")
    values[name] = value

    match name:
        case "hor_distance":
            if values[name] == "Stop":
                sim.stop()
        case "left_speed":
            print(values[name])
        case "right_speed":
            print(values[name])


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        client.subscribe("controller/+")


if __name__ == "__main__":
    client_mqtt = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.on_subscribe = on_subscribe
    client_mqtt.loop_forever()
