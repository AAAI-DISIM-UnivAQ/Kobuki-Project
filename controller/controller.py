import paho.mqtt.client as mqtt

MAX_CORRECTION = 50


class Controller:
    pass


def manage_hor_distance(client, distance):
    print("Received distance: " + str(distance))
    if distance:
        print("IF distance: " + str(distance))
        client.publish(f"controller/hor_distance", "Stop")
    else:
        print("ELSE distance: " + str(distance))
        client.publish(f"controller/hor_distance", "Go")


def manage_correction(client, correction):
    # print("Received correction: ", type(correction))
    left_speed = 2 - correction / MAX_CORRECTION
    right_speed = 2 + correction / MAX_CORRECTION

    client.publish(f"controller/left_speed", left_speed)
    client.publish(f"controller/right_speed", right_speed)


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        client.subscribe("perception/#")


def on_message(client, userdata, msg):
    # values = {}
    perception_name = msg.topic.split("/")[1]
    message_value = msg.payload.decode("utf-8")
    # print("message value", message_value)
    # values[perception_name] = message_value

    match perception_name:
        case "hor_distance":
            # manage_hor_distance(client, values[perception_name])
            if message_value == "True":
                manage_hor_distance(client, True)
            elif message_value == "False":
                manage_hor_distance(client, False)
        case "correction":
            # manage_correction(client, int(values[perception_name]))
            manage_correction(client, int(message_value))


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")


if __name__ == "__main__":
    client_mqtt = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=True)
    client_mqtt.connect("mosquitto", 1883)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.on_subscribe = on_subscribe
    client_mqtt.loop_forever()
