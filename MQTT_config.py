# pip3 install paho-mqtt

#python packages
import paho.mqtt.client as mqtt
import threading
#import time 

#custom modules
import config

#-------------------------MQTT sub/pub--------------------------------------
class port_Topics:
    def __init__(self, name, sub_topics, pub_topics):
        self.name = name
        self.event = threading.Event()
        self.sub_topics = sub_topics
        self.pub_topics = pub_topics
        self.sub_values = {i:0 for i in self.sub_topics}
        self.sub_events = {i:threading.Event() for i in self.sub_topics}
        self.pub_values = {i:0 for i in self.pub_topics}


RS485_port_Topics = port_Topics(name='RS485_port_Topics',
                                sub_topics=[],
                                pub_topics=[
                                    'TC7', 'TC8', 'TC9', 'TC10', # # TC7(ADAM_TC_0), TC8(ADAM_TC_1), TC9(ADAM_TC_2), TC10(ADAM_TC_3) 
                                    'TC11', 'EVA_out', 'RAD_in', 'RAD_out' # TC11(ADAM_TC_4), EVA_out(ADAM_TC_5), RAD_in(ADAM_TC_6), RAD_out(ADAM_TC_7)
                                    ])

RS232_port_Topics = port_Topics(name='RS232_port_Topics',
                                sub_topics=[],
                                pub_topics=[
                                    'GA_CO', 'GA_CO2', 'GA_CH4',
                                    'GA_H2', 'GA_N2', 'GA_HEAT'
                                    ])

Scale_port_Topics = port_Topics(name='Scale_port_Topics',
                                sub_topics=[],
                                pub_topics=[
                                    'Scale'
                                    ])

Setup_port_Topics = port_Topics(name='Setup_port_Topics', 
                                sub_topics=[
                                    'Header_EVA_SV', 'Header_BR_SV', # Header EVA(Header_EVA_SV), Header BR(Header_BR_SV),
                                    'PCB_SET_SV', 'Pump_SET_SV', 'Air_MFC_SET_SV', 'H2_MFC_SET_SV' # PCB(ADAM_SET_SV0), Pump(ADAM_SET_SV1), Air_MFC(ADAM_SET_SV2), H2_MFC(ADAM_SET_SV3)
                                ],
                                pub_topics=[
                                    'Header_EVA_PV', 'Header_BR_PV', # Header EVA(Header_EVA_PV), Header BR(Header_BR_PV),
                                    'PCB_SET_PV', 'Pump_SET_PV', 'Air_MFC_SET_PV', 'H2_MFC_SET_PV', # PCB(ADAM_SET_PV0), Pump(ADAM_SET_PV1), Air_MFC(ADAM_SET_PV2), H2_MFC(ADAM_SET_PV3)
                                    'SMC_0_PV', 'SMC_1_PV', 'ADAM_READ_PV2', 'ADAM_READ_PV3', 'Pump_PV', 'Air_MFC_PV', 'H2_MFC_PV', 'ADAM_READ_PV7' # ADAM_READ_PV0 (SMC), ADAM_READ_PV1 (SMC), ADAM_READ_PV2, ADAM_READ_PV3, ADAM_READ_PV4(pump), ADAM_READ_PV5(Air_MFC), ADAM_READ_PV6(H2_MFC), ADAM_READ_PV7
                                ])

GPIO_port_Topics = port_Topics(name='GPIO_port_Topics',
                                sub_topics=[],
                                pub_topics=[
                                    'DFM_RichGas', 'DFM_AOG'
                                ])

err_Topics = port_Topics(name='err_Topics',
                        sub_topics=[],
                        pub_topics=[
                            'ADAM_TC_collect_err', 'ADAM_TC_analyze_err',
                            'GA_data_collect_err', 'GA_data_analyze_err',
                            'Scale_collect_err', 'Scale_analyze_err',
                            'Header_EVA_collect_err', 'Header_EVA_set_err', 'Header_EVA_analyze_err',
                            'Header_BR_collect_err', 'Header_BR_set_err', 'Header_BR_analyze_err',
                            'ADAM_SET_collect_err', 'ADAM_SET_set_err', 'ADAM_SET_analyze_err',
                            'ADAM_READ_collect_err', 'ADAM_READ_analyze_err',
                            'DFM_data_collect_err', 'DFM_data_analyze_err', 
                            'DFM_AOG_data_collect_err', 'DFM_AOG_data_analyze_err'
                        ])

lst_port_Topics = [RS485_port_Topics, RS232_port_Topics, Scale_port_Topics, Setup_port_Topics, GPIO_port_Topics, err_Topics]

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
            Setup_port_Topics.sub_values[msg.topic] = float(msg.payload)
            Setup_port_Topics.sub_events[msg.topic].set()
        
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
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time):
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