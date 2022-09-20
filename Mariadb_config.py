# pip3 install mariadb
# pip3 install python-dotenv

#python packages
import os
import sys
# import threading
import multiprocessing
import mariadb
from dotenv import load_dotenv
import time

#custom modules
import params
import config

#-------------------------time and tokens--------------------------------------
load_dotenv()
username = os.environ.get("db_user")
password = os.environ.get("db_pwd")
host_vpn = os.environ.get("host_vpn")
#print(username, password)

#-------------------------mariadb conn--------------------------------------
try:
    # Connect to MariaDB Platform
    pool = mariadb.ConnectionPool(
        user=username,
        password=password,
        host=host_vpn,
        port=3306,
        database="reformer",
        autocommit=True
    )

    conn = pool.get_connection()
    # Get Cursor for tx
    cur = conn.cursor()

    #-------------------------create table--------------------------------------
    Table_col = [u + ' FLOAT' for i in config.lst_ports for u in i.pub_topics + i.sub_topics]
    TableSchema = f'create table platform_{config.db_time} (Id int NOT NULL AUTO_INCREMENT,' \
                    + ','.join(Table_col) \
                    + ',PRIMARY KEY (Id))'
    #print(TableSchema)

    try:
        cur.execute(f'drop table if exists platform_{config.db_time}') 
        cur.execute(TableSchema)
        config.db_connection = True
        print('Create tables succeed')       
    except mariadb.Error as e:
        config.db_connection = False
        print(f"Error creating Table: {e}")
        #sys.exit(2)

    #-------------------------db func--------------------------------------
    insert_col = [u for i in config.lst_ports for u in i.pub_topics + i.sub_topics]
    insertSchema = f'INSERT INTO platform_{config.db_time} (' \
                    + ','.join(insert_col) \
                    + f') VALUES ({("?,"*len(insert_col))[:-1]})'
    #print(insertSchema)
except mariadb.Error as e:
    config.db_connection = False
    print(f"Error connecting to MariaDB Platform: {e}")
    #sys.exit(1)
finally:
    conn.close()
    print("After table and scheme creation. Close connection to MariaDB")    


    # def multi_insert(cur):
def multi_insert():
    while not params.kb_event.is_set():
        time.sleep(params.comm_time)
        conn = pool.get_connection()
        cur = conn.cursor()
            # if not params.ticker.wait(params.sample_time):
        try:
            cur.execute(
                insertSchema,
                #tuple(i for i in config.Setup_port.sub_values.values()) + # sub value
                #tuple(u for i in config.lst_ports for u in list(i.pub_values.values()) + list(i.sub_values.values())) # pub value
                tuple(config.NodeRed.get(_k) for _k in insert_col)
                )
            print(f"Successfully added entry to database. Last Inserted ID: {cur.lastrowid}")
        except mariadb.Error as e:
            config.db_connection = False
            print(f"Error adding entry to database: {e}")
        conn.close()
    pool.close()
    print("close connection to MariaDB") 
    #-------------------------main--------------------------------------
#     try:
#         multi_insert = threading.Thread(
#             target=multi_insert,
#             args=(cur,),
#             )
#         multi_insert.start()
#     except mariadb.Error as e:
#         config.db_connection = False
#         print(f"Error multi_insert to MariaDB Platform: {e}")    

# except mariadb.Error as e:
#     config.db_connection = False
#     print(f"Error connecting to MariaDB Platform: {e}")
#     #sys.exit(1)
multi_insert_process = multiprocessing.Process(
    name='multi_insert_process',
    target=multi_insert,
    args=(),
    )
#multi_insert.start()
