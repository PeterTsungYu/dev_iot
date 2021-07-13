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
        database="catalyst",
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
    'TCHeader_SV0 FLOAT,',
    'TCHeader_SV1 FLOAT,',
    'TCHeader_PV0 FLOAT,',
    'TCHeader_PV1 FLOAT,',
    'Scale FLOAT,',
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
    'TCHeader_SV0,',
    'TCHeader_SV1,',
    'ADAM_4024_ch00,',
    'TCHeader_PV0,',
    'TCHeader_PV1,',
    'Scale',
    'ADAM_4024_PV',
    ') ',
    'VALUES (?, ?, ?, ?, ?)'
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
        

def get_data(last_name):
    try:
      cur.execute("SELECT first_name, last_name FROM employees WHERE last_name=?", (last_name,))
      for (first_name, last_name) in cur:
        print(f"Successfully retrieved {first_name}, {last_name}")
    except database.Error as e:
      print(f"Error retrieving entry from database: {e}")

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