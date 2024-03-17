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

def fetch_streamed_image(url):
    """
    Fetch the first image from a multipart HTTP stream.

    Parameters:
    - url: The URL from which to fetch the streamed image.

    Returns:
    - A numpy array representing the image if successful, None otherwise.
    """
    response = requests.get(url, stream=True)

    if response.status_code == 200:
        bytes_data = bytes()
        for chunk in response.iter_content(chunk_size=1024):
            bytes_data += chunk
            # Check if the end of one part of the stream is reached
            end_of_frame = bytes_data.find(b'\r\n\r\n') + 4
            if end_of_frame != -1:
                content_type_index = bytes_data.find(b'Content-Type: image/jpeg\r\n\r\n')
                if content_type_index != -1:
                    # Extract the JPEG image data
                    jpeg_data = bytes_data[content_type_index + len(b'Content-Type: image/jpeg\r\n\r\n'):end_of_frame - 4]
                    image_array = np.frombuffer(jpeg_data, dtype=np.uint8)
                    img = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                    return img
                break
    return None

# Function to process the received message
def process_messages(mq):
    #if first:

    #    first = False
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
            url = f"http://localhost:5000/crop_feed/{sensor_name}?id={image_id}"


            print(f"sensor_name = {sensor_name}, id = {image_id}, url: {url}")
            # Send HTTP request to get the image
            img = fetch_streamed_image(url=url)
            if img is None:
                print(f"Skipped image id {image_id}")
                continue
            # if response.status_code == 200:
            #     # Process the received image (dummy function)
            #     if response.content == b'Sensor key not found':
            #         print(f"skipping msg w id {image_id}")
            #         continue
            objects_detected = process_image(img)

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
