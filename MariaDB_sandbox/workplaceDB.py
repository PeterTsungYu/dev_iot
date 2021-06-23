import os
import sys
import mariadb


username = os.environ.get("db_user")
password = os.environ.get("db_pwd")
#print(username, password)


def add_data(first_name, last_name):
    try:
        cur.execute("INSERT INTO employees (first_name,last_name) VALUES (?, ?)", (first_name, last_name))
        conn.commit()
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


# Connect to MariaDB Platform
try:
    conn = mariadb.connect(
        user=username,
        password=password,
        host="rasp-002.local",
        port=3306,
        database="workplace"
    )
except mariadb.Error as e:
    print(f"Error connecting to MariaDB Platform: {e}")
    sys.exit(1)

# Get Cursor
cur = conn.cursor()

add_data("Kofi", "Doe")
get_data("Doe")

conn.close()