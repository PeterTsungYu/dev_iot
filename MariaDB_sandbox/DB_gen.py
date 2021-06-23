from mysql.connector import connect, Error
from datetime import datetime


time = datetime.now().strftime('%Y_%m_%d_%H_%M')
SQL_File_Name = "Table_Schema.sql"

connected = False
while not connected:
    try:
        with connect(
            host="rasp-002.local",
            user='rasp-001',
            password='rasp-001',
            database="Reformer",
        ) as SQLconn:
            print(SQLconn)
            '''
            with open(SQL_File_Name, 'r') as SchemaFile:
                #Read Table Schema into a Variable and remove all New Line Chars
                TableSchema = SchemaFile.read()
            '''
            TableSchema = [
                f"create table MFC_{time} (Time REAL, Pressure REAL, Temper REAL, VolFlow REAL, MassFlow REAL, Setpoint REAL);",
                f"create table GA_{time} (Time REAL, CO REAL, CO2 REAL, CH4 REAL, H2 REAL, N2 REAL, HEAT REAL);", 
                f"create table Scale_{time} (Time REAL, Weight REAL);", 
                f"create table DFM_{time} (Time REAL, FlowRate REAL);", 
                f"create table ADAM_TC_{time} (Time REAL, TC_7 REAL, TC_8 REAL, TC_9 REAL, TC_10 REAL, TC_11 REAL, TC_12 REAL, TC_13 REAL, TC_14 REAL);"
                ]
            with SQLconn.cursor() as cursor:
                for i in range(len(TableSchema)):
                    cursor.execute(TableSchema[i])
                SQLconn.commit()
            print('Create tables succeed')
            connected = True
    except Error as e:
        print(e)
        connected = False