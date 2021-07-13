# pip3 install paho-mqtt

#python packages
import paho.mqtt.client as mqtt
import threading
#import time 

#custom modules
import config

#-------------------------MQTT sub/pub--------------------------------------
sub_Topics = {
    'TCHeader/SV0':{'value':0, 'event':threading.Event()},
    'TCHeader/SV1':{'value':0, 'event':threading.Event()},
    'ADAM_4024/ch00':{'value':0, 'event':threading.Event()},
}

pub_Topics = {
    'TCHeader/PV0':0,
    'TCHeader/PV1':0,
    'Scale':0,
    'ADAM_4024/ch00':0
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
        if (msg.topic != 'nodered') and (msg.payload != b''):
            sub_Topics[msg.topic]['value'] = float(msg.payload)
            sub_Topics[msg.topic]['event'].set()
        
    def on_publish(client, userdata, mid):
        print(mid)

    client = mqtt.Client(client_id=client_id, clean_session=True)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message
    #client.on_publish = on_publish
    client.connect_async(hostname, port, keepalive)
    return client


def multi_pub(client):
    #global pub_Topics
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time):
            for key, value in pub_Topics.items():
                client.publish(topic=key, payload=value, qos=0, retain=True)

#-------------------------MQTT instance--------------------------------------
client_0 = connect_mqtt(
    client_id='client0', hostname='localhost', port=1883, keepalive=60,) 
client_0.loop_start()

multi_pub = threading.Thread(
    target=multi_pub,
    args=(client_0,),
    )
multi_pub.start()