import paho.mqtt.client as mqtt  # import the client1
import json
import time
import threading

from src import que

# import detect

debug = False
rxMess = None
newMess = False

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_VISERCAM_SUBSCRIBE = "visercam/cmd"
MQTT_TOPIC_VISERCAM_PUBLISH = "visercam/capture"
MQTT_TOPIC_DETECT_PUBLISH = "detect"

debug = True

def on_message(client, userdata, message):
    if debug:
        print("message received ", str(message.payload.decode("utf-8")))
        print("message topic=", message.topic)
        print("message qos=", message.qos)
        print("message retain flag=", message.retain)

    global rxMess
    global newMess
    global topic
    rxMess = message
    newMess = True


def on_disconnect(client, userdata, flags, rc=0):
    m = "DisConnected flags" + "result code " + str(rc) + "client_id  "
    print(m)
    client.connected_flag = False


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK Returned code=", rc)
        client.connected_flag = True  # Flag to indicate success

        client.subscribe(MQTT_TOPIC_VISERCAM_SUBSCRIBE)

    else:
        print("Bad connection Returned code=", rc)
        client.bad_connection_flag = True


def on_log(client, userdata, level, buf):
    print("log: ", buf)


class MqttInterface:
    def __init__(self, broker_address='localhost', port=1883, username='user', password='ssch', client_name="visersim"):

        self.broker_address = broker_address
        self.port = port
        self.keep_alive = 60

        mqtt.Client.connected_flag = False  # create flags
        mqtt.Client.bad_connection_flag = False  #
        mqtt.Client.retry_count = 0  #
        # pdb.set_trace()

        self.client = mqtt.Client(client_name)  # create new instance
        self.client.on_connect = on_connect  # attach function to callback
        self.client.on_disconnect = on_disconnect
        self.client.on_message = on_message
        # if username != 'user' and password != 'ssch':
        self.client.username_pw_set(username=username, password=password)

        self.activeCameras = []
        self.activeDetection = True
        self.processMessage = False
        self.det = None
        self.youve_got_post = False
        self.msg_queue = que.ImageQueue(max_size=30, name="detection message queue")

        thr = threading.Thread(target=self._checkMessage)
        thr.daemon = True
        thr.start()

        self.thrDet = None

    def _checkMessage(self):
        global newMess
        while True:

            if self.client.connected_flag:

                if self.det != None:
                    if self.det.alert.sendMqttMessage:
                        print('received mqtt message request')
                        self.det.alert.sendMqttMessage = False
                        #self.sendMessage(variable='detections', value=str(self.det.alert.mqttTrigCnt))
                        #self.sendMessage(variable='DETECTION', value=self.det.alert.mqttAlertDetected)
                        self.det.alert.mqttTrigCnt = 0

                if newMess:
                    newMess = False
                    #print("message received ", str(rxMess.payload.decode("utf-8")))
                    print("message topic=", rxMess.topic)
                    #print("message qos=", rxMess.qos)
                    #print("message retain flag=", rxMess.retain)
                    data = json.loads(rxMess.payload.decode("utf-8"))
                    if "cmd" in rxMess.topic:
                        command = data["cmd"]
                        if command == "ON":
                            print('Turn on camera capture')
                            self.activeCameras = True
                            self.activeDetection = True
                        elif command == "OFF":
                            print('Turn off camera capture')
                            self.activeCameras = False
                            self.activeDetection = False

                    elif "visercam/capture/" in rxMess.topic:
                        self.msg_queue.push(rxMess)

            time.sleep(0.1)

    def connect(self):

        while not self.client.connected_flag and self.client.retry_count < 3:
            self.count = 0
            try:
                print("connecting ", self.broker_address)
                self.client.connect(self.broker_address, self.port, self.keep_alive)  # connect to broker
                return True
            except:
                print("connection attempt failed will retry")
                self.client.retry_count += 1
                time.sleep(1)
                if self.client.retry_count > 3:
                    return False

    def startLoop(self):

        while True:
            self.client.loop_start()
            if self.client.connected_flag:  # wait for connack
                self.client.retry_count = 0  # reset counter
                self.connected = True
                return True
            if self.count > 6 or self.client.bad_connection_flag:  # don't wait forever
                self.client.loop_stop()  # stop loop
                self.client.retry_count += 1
                if self.client.retry_count > 3:
                    self.connected = False
                return False  # break from while loop

            time.sleep(1)
            self.count += 1

    def subscribe(self, topic='cmnd/detect/person/ENABLED'):
        if debug:
            print(f"subscribing to topic {topic}")
        self.client.subscribe(topic)

    def publish_detection_result(self, topic, detection_result):

        self.client.publish(topic, json.dumps(detection_result))
        print("Published detection result.")

    # this is used by the entity that captures the images and crops out a section for detection
    def publish_capture_result(self, name, datetime, id, meta):
        (x1, y1), (x2, y2) = meta['cropouts'][0]
        result = {
            "time": datetime,
            "name": name,
            "id": id,
            "x1": int(x1),
            "x2": int(x2),
            "y1": int(y1),
            "y2": int(y2)

        }
        self.client.publish(f'{MQTT_TOPIC_VISERCAM_PUBLISH}/cam_{name}', json.dumps(result))
        #print("Published capture result.")

    def disconnect(self):
        print("quitting")
        self.client.disconnect()
        self.client.loop_stop()
        self.det.runFlag = False


# CLEAN_SESSION=False
