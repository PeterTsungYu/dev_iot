# pip3 install mariadb
# pip3 install python-dotenv

#python packages
import os
import sys
import threading
import mariadb
from datetime import datetime
from dotenv import load_dotenv

#custom modules
import config
import MQTT_config

#-------------------------time and tokens--------------------------------------
time = datetime.now().strftime('%Y_%m_%d_%H')
load_dotenv()
username = os.environ.get("db_user")
password = os.environ.get("db_pwd")
print(username, password)

#-------------------------mariadb conn--------------------------------------
try:
    # Connect to MariaDB Platform
    conn = mariadb.connect(
        user=username,
        password=password,
        host="rasp-002.local",
        port=3306,
        database="reformer",
        autocommit=True
    )

    # Get Cursor for tx
    cur = conn.cursor()
except mariadb.Error as e:
    print(f"Error connecting to MariaDB Platform: {e}")
    sys.exit(1)

#-------------------------create table--------------------------------------
TableSchema = [
    f'create table platform_{time} (',
    'Id int NOT NULL AUTO_INCREMENT,',
    'TCHeader_0_SV FLOAT,',
    'TCHeader_1_SV FLOAT,',
    'ADAM_SET_SV0 FLOAT,',
    'ADAM_SET_SV1 FLOAT,',
    'ADAM_SET_SV2 FLOAT,',
    'ADAM_SET_SV3 FLOAT,',
    'TCHeader_0_PV FLOAT,',
    'TCHeader_1_PV FLOAT,',
    'Scale FLOAT,',
    'ADAM_SET_PV0 FLOAT,',
    'ADAM_SET_PV1 FLOAT,',
    'ADAM_SET_PV2 FLOAT,',
    'ADAM_SET_PV3 FLOAT,',
    'ADAM_READ_PV0 FLOAT,',
    'ADAM_READ_PV1 FLOAT,',
    'ADAM_READ_PV2 FLOAT,',
    'ADAM_READ_PV3 FLOAT,',
    'ADAM_READ_PV4 FLOAT,',
    'ADAM_READ_PV5 FLOAT,',
    'ADAM_READ_PV6 FLOAT,',
    'ADAM_READ_PV7 FLOAT,',
    'ADAM_TC_PV0 FLOAT,',
    'ADAM_TC_PV1 FLOAT,',
    'ADAM_TC_PV2 FLOAT,',
    'ADAM_TC_PV3 FLOAT,',
    'ADAM_TC_PV4 FLOAT,',
    'ADAM_TC_PV5 FLOAT,',
    'ADAM_TC_PV6 FLOAT,',
    'ADAM_TC_PV7 FLOAT,',
    'GA_CO FLOAT,',
    'GA_CO2 FLOAT,',
    'GA_CH4 FLOAT,',
    'GA_H2 FLOAT,',
    'GA_N2 FLOAT,',
    'GA_HEAT FLOAT,',
    'DFM_RichGas FLOAT,',
    'DFM_AOG FLOAT,',
    'PRIMARY KEY (Id)',
    ')'
    ]
try:
    cur.execute(f'drop table if exists platform_{time}') 
    cur.execute(''.join(TableSchema)) 
    print('Create tables succeed')       
except mariadb.Error as e:
    print(f"Error creating Table: {e}")
    sys.exit(2)

#-------------------------db func--------------------------------------
insertSchema = [
    f'INSERT INTO platform_{time} (',
    'TCHeader_0_SV,',
    'TCHeader_1_SV,',
    'ADAM_SET_SV0,',
    'ADAM_SET_SV1,',
    'ADAM_SET_SV2,',
    'ADAM_SET_SV3,',
    'TCHeader_0_PV,',
    'TCHeader_1_PV,',
    'Scale,',
    'ADAM_SET_PV0',
    'ADAM_SET_PV1',
    'ADAM_SET_PV2',
    'ADAM_SET_PV3',
    'ADAM_READ_PV0',
    'ADAM_READ_PV1',
    'ADAM_READ_PV2',
    'ADAM_READ_PV3',
    'ADAM_READ_PV4',
    'ADAM_READ_PV5',
    'ADAM_READ_PV6',
    'ADAM_READ_PV7',
    'ADAM_TC_PV0',
    'ADAM_TC_PV1',
    'ADAM_TC_PV2',
    'ADAM_TC_PV3',
    'ADAM_TC_PV4',
    'ADAM_TC_PV5',
    'ADAM_TC_PV6',
    'ADAM_TC_PV7',
    'GA_CO',
    'GA_CO2',
    'GA_CH4',
    'GA_H2',
    'GA_N2',
    'GA_HEAT',
    'DFM_RichGas',
    'DFM_AOG',
    ') ',
    'VALUES (?, ?, ?, ?, ?, ?, ?)'
    ]
def multi_insert(cur):
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time):
            try:
                cur.execute(
                    ''.join(insertSchema), 
                    tuple(i['value'] for i in MQTT_config.sub_Topics.values()) + 
                    tuple(MQTT_config.pub_Topics.values())
                    )
                print(f"Successfully added entry to database. Last Inserted ID: {cur.lastrowid}")
            except mariadb.Error as e:
                print(f"Error adding entry to database: {e}")

#-------------------------main--------------------------------------
multi_insert = threading.Thread(
    target=multi_insert,
    args=(cur,),
    )
multi_insert.start()

'''
while True:
    pass
'''