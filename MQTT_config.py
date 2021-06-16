# pip3 install paho-mqtt
import paho.mqtt.client as mqtt
import threading
import time 

#-------------------------MQTT var--------------------------------------
sub_Topics = {
    'TCHeader/SV0':{'value':0, 'event':threading.Event()},
    'TCHeader/SV1':{'value':0, 'event':threading.Event()},
}

#-------------------------MQTT func--------------------------------------
def connect_mqtt(client_id, hostname='localhost', port=1883, keepalive=60,):
    def on_connect(client, userdata, flags, rc):
        global sub_Topics
        if rc == 0:
            print(f"Connected to MQTT!" + f">>> {client_id}")
        else:
            print(f"{client_id} Failed to connect, return code %d\n", rc)
        
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe("nodered", qos=0)
        client.subscribe([(i,0) for i in sub_Topics.keys()])
        #client.subscribe([("TCHeader/SV0", 0), ("TCHeader/SV1", 0)])

    # The callback for when a SUB message is received
    def on_message(client, userdata, msg):
        global sub_Topics
        print(msg.topic+ ": " + str(msg.payload) + f">>> {client_id}")
        if msg.topic != 'nodered':
            sub_Topics[msg.topic]['value'] = float(msg.payload)
            sub_Topics[msg.topic]['event'].set()
        
        """ if msg.topic == "TCHeader/SV0":
            globals()[sub[sub.index('sub_SV0')]] = float(msg.payload)
            #print(type(globals()[sub]))
            globals()[sub_event[sub_event.index('sub_SV0_event')]].set()
        elif msg.topic == "TCHeader/SV1":
            globals()[sub[sub.index('sub_SV1')]] = float(msg.payload)
            globals()[sub_event[sub_event.index('sub_SV1_event')]].set() """


    client = mqtt.Client(client_id=client_id, clean_session=True)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect_async(hostname, port, keepalive)
    return client


#-------------------------MQTT instance--------------------------------------
client_0 = connect_mqtt(
    client_id='client0', hostname='localhost', port=1883, keepalive=60,) 
client_0.loop_start()


#for testing MQTT
""" while True:
    print(sub_SV0, sub_SV1)
    print(sub_SV0_event.isSet(), sub_SV1_event.isSet())
    time.sleep(2) """