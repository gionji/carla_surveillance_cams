import time

import paho.mqtt.client as mqtt
import requests
from datetime import datetime
import json
from src import mqtt, rest, objects
from hemligt import *
import cv2

# Function to process the received message
def process_messages(mq):
    #if first:

    #    first = False
    while not mq.msg_queue.is_empty():
        try:
            print("in!")
            msg = mq.msg_queue.pop_oldest()
            if msg is None:
                continue
            data = json.loads(msg.payload.decode())
            sensor_name = data["name"]
            image_id = data["id"]

            # Construct URL for image request
            url = f"http://localhost:5000/crop_feed/{sensor_name}?id={image_id}"

            print(f"sensor_name = {sensor_name}, id = {image_id}, url: {url}")
            # Send HTTP request to get the image
            response = requests.get(url)
            if response.status_code == 200:
                # Process the received image (dummy function)
                if response.content == b'Sensor key not found':
                    print(f"skipping msg w id {image_id}")
                    continue
                objects_detected = process_image(response.content)

                # Prepare data for publishing
                result = {
                    "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                    "ID": image_id,
                    "objects_detected": objects_detected
                }

                # Publish result to MQTT topic
                mq.publish_detection_result(topic=f"viserdetect/{sensor_name}/", detection_result=result)
            else:
                print(f"Failed to fetch image: {response.status_code}")
        except Exception as e:
            print(f"Error processing message: {e}")


# Function to process the received image (dummy function)
def process_image(image):
    cv2.imshow(f"Crop from sensor", image)
    cv2.waitKey(10)
    # Your image processing logic here
    # This is a dummy function returning a list of detected objects
    return ["object1", "object2", "object3"]


# MQTT settings



def setup_mqtt():
    broker_address = "localhost"
    port = 1883
    topic = "visercam/capture/#"

    mq = mqtt.MqttInterface(broker_address='localhost', port=1883, username=C_USER, password=C_PASS, client_name="detection", q_size=100)


    global runFlag
    runFlag = False
    if mq.connect():
        if mq.startLoop():
            print("MQTT connection established.")

    mq.subscribe(topic=topic)
    return mq


if __name__ == '__main__':
    mq = setup_mqtt()

    first = True

    while True:
        if mq.activeDetection:
            #for c in mq.msg_queue.inspect():
            #    print(c)
            process_messages(mq)
            # mq.youve_got_post = False

        time.sleep(0.1)
