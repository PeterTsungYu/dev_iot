#python packages
import os
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
def create_pool():
    # Connect to MariaDB Platform
    pool = mariadb.ConnectionPool(
        user=username,
        password=password,
        host=host_vpn,
        port=3306,
        database="reformer",
        autocommit=True,
        pool_name='conn_pool',
        pool_size = 3, 
    )
    return pool    


def multi_insert(pool):
    while not params.kb_event.is_set():
        time.sleep(params.comm_time)
        try:
            conn = pool.get_connection()
            cur = conn.cursor()
            if config.db_table:
                cur.execute(
                    insertSchema,
                    tuple(config.NodeRed.get(_k) for _k in insert_col)
                    )
                print(f"Successfully added entry to database. Last Inserted ID: {cur.lastrowid}")
            else:
                cur.execute(TableSchema)
                config.db_table = True
                print('Create tables succeed')       
            conn.close()
        except mariadb.Error as e:
            print(f"Trying to reconnect after error: {e}")
            if not isinstance(pool, mariadb.ConnectionPool):
                response = os.system("ping -c 1 " + host_vpn)
                if response == 0:
                    pool = create_pool()
                
    if isinstance(pool, mariadb.ConnectionPool):
        pool.close()
    print("close connection to MariaDB") 

#-------------------------main--------------------------------------
config.db_table = False
pool = None

Table_col = [u + ' FLOAT' for i in config.lst_ports for u in i.pub_topics + i.sub_topics]
TableSchema = f'create table platform_{config.db_time} (Id int NOT NULL AUTO_INCREMENT,' \
                + ','.join(Table_col) \
                + ',PRIMARY KEY (Id))'
#print(TableSchema)

insert_col = [u for i in config.lst_ports for u in i.pub_topics + i.sub_topics]
insertSchema = f'INSERT INTO platform_{config.db_time} (' \
                + ','.join(insert_col) \
                + f') VALUES ({("?,"*len(insert_col))[:-1]})'
#print(insertSchema)

response = os.system("ping -c 1 " + host_vpn)
if response == 0:
    pool = create_pool()
    conn = pool.get_connection()
    cur = conn.cursor()
    try:
        cur.execute(f'drop table if exists platform_{config.db_time}') 
        cur.execute(TableSchema)
        config.db_table = True
        print('Create tables succeed')       
    except mariadb.Error as e:
        config.db_table = False
        print(f"Error creating Table: {e}")
    conn.close()

multi_insert_process = multiprocessing.Process(
    name='multi_insert_process',
    target=multi_insert,
    args=(pool),
    )
