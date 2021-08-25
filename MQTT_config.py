# pip3 install paho-mqtt

#python packages
import paho.mqtt.client as mqtt
import threading
#import time 

#custom modules
import params
import config

#-------------------------MQTT sub/pub--------------------------------------
lst_port_Topics = [config.RS485_port, config.RS232_port, config.Scale_port, config.Setup_port, config.GPIO_port]

#-------------------------MQTT func--------------------------------------
def connect_mqtt(client_id, hostname='localhost', port=1883, keepalive=60,):
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT!" + f">>> {client_id}")
        else:
            print(f"{client_id} Failed to connect, return code %d\n", rc)
        
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe("nodered", qos=0)
        client.subscribe([(u,0) for i in lst_port_Topics for u in i.sub_topics])
        #client.subscribe([("Header_EVA_SV", 0), ("Header_BR_SV", 0)])

    # The callback for when a SUB message is received
    def on_message(client, userdata, msg):
        print(msg.topic+ ": " + str(msg.payload) + f">>> {client_id}")
        if (msg.topic != 'nodered') and (msg.payload != b''):
            config.Setup_port.sub_values[msg.topic] = float(msg.payload)
            config.Setup_port.sub_events[msg.topic].set()
        
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
    while not params.kb_event.isSet():
        if not params.ticker.wait(params.sample_time):
            #print(pub_Topics)
            for port_topic in lst_port_Topics:
                for key, value in port_topic.pub_values.items():
                    client.publish(topic=key, payload=value, qos=0, retain=True)
            print(f"pub succeed {client._client_id} >>> localhost")

#-------------------------MQTT instance--------------------------------------
client_0 = connect_mqtt(
    client_id='client0', hostname='localhost', port=1883, keepalive=60,) 
client_0.loop_start()

multi_pub = threading.Thread(
    target=multi_pub,
    args=(client_0,),
    )
multi_pub.start()