# pip3 install paho-mqtt

#python packages
import paho.mqtt.client as mqtt
import threading
import json

#custom modules
import params
import config

#-------------------------MQTT func--------------------------------------
def connect_mqtt(client_id, hostname='localhost', port=1883, keepalive=60,):
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT!")
        else:
            print(f"Failed to connect, return code %d\n", rc)
        
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe("nodered_port", qos=0)
        #client.subscribe([(u,0) for i in config.lst_ports for u in i.sub_topics])
        #client.subscribe([("Header_EVA_SV", 0), ("Header_BR_SV", 0)])

    # The callback for when a SUB message is received
    def on_message(client, userdata, msg):
        #print(msg.topic+ ": " + str(msg.payload) + f">>> {client_id}")
        resp = json.loads(msg.payload.decode('utf-8'))
        for key, value in resp.items():    
            if (key != 'nodered') and (value != None):
                print(key, value)
                config.Setup_port.sub_values[key] = float(value)
                config.Setup_port.sub_events[key].set()
        
    def on_publish(client, userdata, mid):
        print(mid)

    client = mqtt.Client(client_id=client_id, clean_session=True)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    #client.subscribe([(u,0) for i in config.lst_ports for u in i.sub_topics])
    #client.on_publish = on_publish
    client.on_message = on_message
    client.connect_async(hostname, port, keepalive)
    return client


def multi_pub(client):
    while not params.kb_event.isSet():
        if not params.ticker.wait(params.sample_time):
            for device_port in config.lst_ports:
                payload = {}
                for key, value in device_port.pub_values.items():
                    payload[key] = value
                payload = json.dumps(payload)
                client.publish(topic=f"{device_port.name}", payload=payload, qos=0, retain=True)
                #print(f"pub f'{device_port.name}_pub':{payload} succeed from {client._client_id} >>> localhost")

#-------------------------MQTT instance--------------------------------------
client_0 = connect_mqtt(client_id='client_0' ,hostname='localhost', port=1883, keepalive=60,) 
client_0.loop_start()

multi_pub = threading.Thread(
    target=multi_pub,
    args=(client_0,),
    )
multi_pub.start()