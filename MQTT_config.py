# pip3 install paho-mqtt
import paho.mqtt.client as mqtt
import threading


#-------------------------MQTT var--------------------------------------
topic_ADAM_TC = [
    "/rpi/Reformer_TC_07", "/rpi/Reformer_TC_08", "/rpi/Reformer_TC_09", "/rpi/Reformer_TC_10",
    "/rpi/Reformer_TC_11", "/rpi/Reformer_TC_12", "/rpi/Reformer_TC_13", "/rpi/Reformer_TC_14"]

# set value
sub_SV0 = 0 #md: subscription value and rtu
# incoming msg event
sub_SV0_event = threading.Event()


#-------------------------MQTT func--------------------------------------
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


#-------------------------MQTT instance--------------------------------------
client_0 = connect_mqtt(client_id='client0', hostname='localhost', port=1883, keepalive=60, sub_topic=[("TCHeader/SV0", 0),], sub="sub_SV0", sub_event="sub_SV0_event",)
client_0.loop_start()
