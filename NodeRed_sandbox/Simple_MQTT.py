# pip3 install crccheck
# pip3 install numpy 
## for numpy, also do this: sudo apt-get install libatlas-base-dev
# pip3 install pyserial
# pip3 install paho-mqtt

from crccheck.crc import Crc16Modbus
import numpy as np
import serial
import time
import threading
import re
import signal
import paho.mqtt.client as mqtt
import random


#-------------------------Global Vars--------------------------------------
sub_SV0 = 0
sub_SV1 = 0

#-------------------------Threading Events--------------------------------------
# incoming msg event
sub_SV0_event = threading.Event()
sub_SV1_event = threading.Event()

# Keyboard interrupt event to kill all the threads
kb_event = threading.Event()
def signal_handler(signum, frame):
    kb_event.set()
signal.signal(signal.SIGINT, signal_handler) # Keyboard interrupt to stop the program

#-------------------------MQTT Conn and Sub--------------------------------------
def connect_mqtt(client_id, hostname='localhost', port=1883, keepalive=60, sub_topic=[("", 0),], sub="", sub_event=""):
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT!" + f">>> {client_id}")
        else:
            print(f"{client_id} Failed to connect, return code %d\n", rc)
        
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe("nodered", qos=0)
        client.subscribe(sub_topic)
        #client.subscribe([("TCHeader/SV0", 0), ("TCHeader/SV1", 0)])

    # The callback for when a SUB message is received
    def on_message(client, userdata, msg):
        print(msg.topic+ ": " + str(msg.payload) + f">>> {client_id}")
        if msg.topic != "nodered":
            globals()[sub] = float(msg.payload)
            #print(type(globals()[sub]))
            globals()[sub_event].set()


    client = mqtt.Client(client_id=client_id, clean_session=True)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect_async(hostname, port, keepalive)
    return client

client_0 = connect_mqtt(client_id='client0', hostname='localhost', port=1883, keepalive=60, sub_topic=[("TCHeader/SV0", 0),], sub="sub_SV0", sub_event="sub_SV0_event",)
client_0.loop_start()
#print(threading.enumerate())
client_1 = connect_mqtt(client_id='client1', hostname='localhost', port=1883, keepalive=60, sub_topic=[("TCHeader/SV1", 0),], sub="sub_SV1", sub_event="sub_SV1_event",)
client_1.loop_start()
#print(threading.enumerate())

#-------------------------MQTT Pub--------------------------------------
lst_thread = []

def pub_0(client=client_0):
    while True:
        client.publish(topic='TCHeader/PV0', payload=random.random(), qos=0, retain=True)
        time.sleep(2)
def pub_1(client=client_1):
    while True:
        client.publish(topic='TCHeader/PV1', payload=random.random(), qos=0, retain=True)
        time.sleep(2)

payload_0 = random.random()
payload_1 = random.random()
pub_0 = threading.Thread(
    target=pub_0, 
    args=(client_0,)
    )
pub_1 = threading.Thread(
    target=pub_1, 
    args=(client_1,)
    )
lst_thread.append(pub_0)
lst_thread.append(pub_1)

#-------------------------Talk to sensors--------------------------------------
def TCHeader_0():
    while not kb_event.isSet():
        time.sleep(1)
        if not sub_SV0_event.isSet():
            print("Collect data from TCHeader_0")
        else:
            print(f"Set TCHeader_0 to {sub_SV0}")
            sub_SV0_event.clear()

def TCHeader_1():
    while not kb_event.isSet():
        time.sleep(1)
        if not sub_SV1_event.isSet():
            print("Collect data from TCHeader_1")
        else:
            print(f"Set TCHeader_1 to {sub_SV1}")
            sub_SV1_event.clear()

TCHeader_0 = threading.Thread(
    target=TCHeader_0, 
    args=()
    )
TCHeader_1 = threading.Thread(
    target=TCHeader_1, 
    args=()
    )
lst_thread.append(TCHeader_0)
lst_thread.append(TCHeader_1)

#-------------------------Start threads--------------------------------------
for subthread in lst_thread:
    subthread.start()
    #print(threading.enumerate())

while True:
    #print(threading.active_count())
    #print(threading.enumerate())
    #print(threading.get_ident())
    time.sleep(1)