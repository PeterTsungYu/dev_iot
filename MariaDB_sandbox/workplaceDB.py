import os
import sys
import mariadb
from datetime import datetime

time = datetime.now().strftime('%Y_%m_%d_%H')
username = os.environ.get("db_user")
password = os.environ.get("db_pwd")
#print(username, password)

TableSchema = [
    'create table person (',
    'Id int NOT NULL AUTO_INCREMENT,',
    'LastName varchar(255) NOT NULL,',
    'FirstName varchar(255),',
    'PRIMARY KEY (Id)',
    ')'
    ]

try:
    # Connect to MariaDB Platform
    conn = mariadb.connect(
        user=username,
        password=password,
        host="rasp-002.local",
        port=3306,
        database="workplace",
        autocommit=True
    )
    # Get Cursor for tx
    cur = conn.cursor()
except mariadb.Error as e:
    print(f"Error connecting to MariaDB Platform: {e}")
    sys.exit(1)

try:
    cur.execute(''.join(TableSchema))
    #conn.commit()   
    print('Create tables succeed')       
except mariadb.Error as e:
    print(f"Error creating Table: {e}")


def add_data(first_name, last_name):
    try:
        cur.execute(f"INSERT INTO person (FirstName, LastName) VALUES (?, ?)", (first_name, last_name))
        #conn.commit()
        print(f"Successfully added entry to database. Last Inserted ID: {cur.lastrowid}")
    except mariadb.Error as e:
        print(f"Error adding entry to database: {e}")
        

def get_data(last_name):
    try:
      cur.execute(f"SELECT FirstName, LastName FROM person WHERE LastName=?", (last_name,))
      for (first_name, last_name) in cur:
        print(f"Successfully retrieved {first_name}, {last_name}")
    except database.Error as e:
      print(f"Error retrieving entry from database: {e}")


add_data("Kofi", "Doe")
#get_data("Doe")

cur.execute("INSERT INTO person(LastName)VALUES('Testing3')")
conn.commit()

conn.close()