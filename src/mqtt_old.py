import json
#import requests
import paho.mqtt.client as mqtt
import rest
import threading

# Constants
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_VISERCAM_SUBSCRIBE = "visercam/cmd"
MQTT_TOPIC_VISERCAM_PUBLISH = "visercam/capture/"
MQTT_TOPIC_DETECT_PUBLISH = "detect/"

REST_API_ENDPOINT = "localhost/images/"

# Placeholder for the detection function
def detect_image(image_data):
    # Implement your detection logic here
    # Return a tuple of (class, box_coordinates)
    x1, y1, x2, y2 = 0, 0, 0, 0
    object_class = 0
    return object_class, (x1, y1, x2, y2)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(MQTT_TOPIC_VISERCAM_SUBSCRIBE)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(f"Message received: {msg.payload}")
    try:
        command = json.loads(msg.payload)
        image_id = command.get("image_id")

        if image_id:
            response = rest.get(REST_API_ENDPOINT + str(image_id))
            if response.status_code == 200:
                detected_class, box_coords = detect_image(response.content)
                publish_detection_result(client, detected_class, box_coords)
            else:
                print(f"Failed to get image: {response.status_code}")
        else:
            print("No image ID in the message")
    except Exception as e:
        print(f"Error: {e}")

def publish_detection_result(client, detected_class, box_coords):
    detection_result = {
        "class": detected_class,
        "box_coordinates": box_coords
    }
    client.publish(MQTT_TOPIC_DETECT_PUBLISH, json.dumps(detection_result))
    print("Published detection result.")

def publish_capture_result(client, result, name):
    client.publish(f'{MQTT_TOPIC_VISERCAM_PUBLISH}/cam_{name}', json.dumps(result))
    print("Published capture result.")


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Blocking call that processes network traffic, dispatches callbacks and handles reconnecting.
client.loop_forever()
