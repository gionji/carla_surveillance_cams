import time

import paho.mqtt.client as mqtt
import requests
from datetime import datetime
import json
from src import mqtt, rest, objects
from hemligt import *
import cv2

import numpy as np
import cv2


from ultralytics import YOLO


def fetch_image_from(base_url, sensor, id):
    """
    Parameters:
    - url: The URL from which to fetch the streamed image.

    Returns:
    - An image as a numpy array if successful, None otherwise.
    """

    url = f"{base_url}/{sensor}"
    params = {'id': id}
    print(f"sensor_name = {sensor}, id = {id}, url: {url}")

    response = requests.get(url, params=params)
    if response.status_code == 200:
        # Convert the response content into a NumPy array
        img_array = np.asarray(bytearray(response.content), dtype=np.uint8)

        # Decode the array into an image
        image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        return image
    else:
        print("Failed to fetch the image. Status code:", response.status_code)
        return None


# Function to process the received message
def process_messages(mq):
    #if first:

    #    first = False
    while mq.msg_queue.is_empty():
        time.sleep(0.1)

    time.sleep(1)

    while not mq.msg_queue.is_empty():
        try:
            #print("in!")
            msg = mq.msg_queue.pop_oldest()
            if msg is None:
                continue
            data = json.loads(msg.payload.decode())
            sensor_name = data["name"]
            image_id = data["id"]

            # Construct URL for image request
            #url = f"http://localhost:5000/crop_feed/{sensor_name}?id={image_id}"
            #print(f"sensor_name = {sensor_name}, id = {image_id}, url: {url}")

            image = fetch_image_from(base_url="http://localhost:5000/crop_feed", sensor=sensor_name, id=image_id)
            if image is not None:

                # Send HTTP request to get the image
                objects_detected = process_image(image)

                # Prepare data for publishing
                result = {
                    "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                    "ID": image_id,
                    "objects_detected": objects_detected
                }

                # Publish result to MQTT topic
                mq.publish_detection_result(topic=f"viserdetect/{sensor_name}/", detection_result=result)

        except Exception as e:
            print(f"Error processing message: {e}")



## https://docs.ultralytics.com/usage/python/#predict
# Function to process the received image (dummy function)
def process_image(image):
    cv2.imshow(f"Crop from sensor", image)
    cv2.waitKey(10)
    # Your image processing logic here
    # This is a dummy function returning a list of detected objects

    results = model.predict(source=image, save=True, save_txt=True)  # save predictions as labels

    return results

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

    # Load a pretrained YOLO model (recommended for training)
    model = YOLO('yolov8n.pt')

    first = True

    while True:
        if mq.activeDetection:
            #for c in mq.msg_queue.inspect():
            #    print(c)
            process_messages(mq)
            # mq.youve_got_post = False

        time.sleep(0.1)
