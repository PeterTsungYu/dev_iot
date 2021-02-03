import sqlite3

#Settings 
#DB Name
DB_NAME = "test_DB.db"

#SQL File with Table Schema and Initialization Data
SQL_File_Name = "Table_Schema.sql"
#=============================================================#

#Read Table Schema into a Variable and remove all New Line Chars

with open(SQL_File_Name, 'r') as SchemaFile:
    #print(SchemaFile.read())
    TableSchema=SchemaFile.read()

#Connect or Create DB File
conn = sqlite3.connect(DB_NAME)
curs = conn.cursor()

#Create Tables
if sqlite3.complete_statement(TableSchema):
    curs.executescript(TableSchema)

#Close DB
curs.close()
conn.commit()
conn.close()