# pip3 install paho-mqtt

#python packages
import paho.mqtt.client as mqtt
# import threading
import multiprocessing
import json
import time

#custom modules
import params
import config

def parse_nested_dicts(d: dict, _d: dict):
    for k, v in d.items():
        if isinstance(v, dict):
            parse_nested_dicts(v, _d)
        else:
            _d[k] = v
#-------------------------MQTT func--------------------------------------
def connect_mqtt(client_id, hostname='localhost', port=1883, keepalive=60,):
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT!")
        else:
            print(f"Failed to connect, return code %d\n", rc)
        
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe("Set_bit", qos=0)
        client.subscribe("NodeRed", qos=0)
        client.subscribe("MFC_Set", qos=0)
        client.subscribe("PID_Set", qos=0)
        client.subscribe("PWM_Set", qos=0)
        #client.subscribe([(u,0) for i in config.lst_ports for u in i.sub_topics])
        #client.subscribe([("Header_EVA_SV", 0), ("Header_BR_SV", 0)])

    # The callback for when a SUB message is received
    def on_message(client, userdata, msg):
        #print(msg.topic+ ": " + str(msg.payload) + f">>> {client_id}")
        resp = json.loads(msg.payload.decode('utf-8'))
        #print(resp)
        try:
            if (msg.topic == 'NodeRed'):
                print(f'{hostname} Receive topic: NodeRed')
                for key, value in resp.items():
                    if type(value) == dict:
                        for k, v in resp[key].items():
                            config.NodeRed[k] = v
                    else:
                        config.NodeRed[key] = value

                # print(config.NodeRed)
            else:
                # print(resp)
                port = None
                if (msg.topic == 'Set_bit'):
                    print(f'{hostname} Receive topic: Set_bit')
                    if config.port_path_dict.get('Setup_port_path'):
                        port = config.Setup_port
                elif (msg.topic == 'MFC_Set'):
                    print(f'{hostname} Receive topic: MFC_Set')
                    if config.port_path_dict.get('RS232_port_path'):
                        port = config.RS232_port
                elif (msg.topic == "PWM_Set"):
                    print(f'{hostname} Receive topic: PWM_Set')
                    port = config.GPIO_port
                elif (msg.topic == 'PID_Set'):
                    print(f'{hostname} Receive topic: PID_Set')
                    # print(resp)
                    if config.port_path_dict.get('PID_port'):
                        port = config.PID_port
                if port:
                    for key, value in resp.items():
                        print(key, value)
                        if port.sub_values.get(key) != None:
                            if port.sub_values[key].value != float(value):
                                port.sub_values[key].value = float(value)
                                port.sub_events[key].set()
        except Exception as e:
            print(f"Error mqtt on_message: {e}")
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


def multi_pub():
    client_mqtt = connect_mqtt(client_id='client_mqtt' ,hostname='localhost', port=1883, keepalive=60,) 
    client_mqtt.loop_start()
    while not params.kb_event.is_set():
        # if not params.ticker.wait(params.sample_time):
        time.sleep(params.comm_time)
        for device_port in config.lst_ports:
            #print(device_port.name)
            for _slave in device_port.slaves:
                payload = {}
                #print(_slave.name)
                for _topic in _slave.port_topics.pub_topics:
                    payload[_topic] = device_port.pub_values[_topic].value
                payload = json.dumps(payload)
                # print(f'pub_{payload}')
                client_mqtt.publish(topic=_slave.name, payload=payload, qos=0, retain=False)
                    #print(f"pub {_slave.name}:{payload} succeed from {client._client_id} >>> localhost")
            # client.publish(topic='DFM_total', payload=config.GPIO_port.pub_values['DFM_RichGas'] + config.GPIO_port.pub_values['DFM_AOG'], qos=0, retain=False)
        if config.db_table == True:
            client_mqtt.publish(topic='DB_name', payload=config.db_time, qos=0, retain=False)
        elif config.db_table == False:
            client_mqtt.publish(topic='DB_name', payload='', qos=0, retain=False)
    client_mqtt.loop_stop()
    client_mqtt.disconnect()
    print("close connection to MQTT broker")

#-------------------------MQTT instance--------------------------------------

multi_pub_process = multiprocessing.Process(
    name='multi_pub_mqtt',
    target=multi_pub,
    args=(),
    )
# multi_pub.start()
