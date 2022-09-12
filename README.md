# Pythonic IoT Project
## Scope
The aims of this IoT project are to establish a framework for the below...
- Collect and analyze data
- Remotely automatic control
- Pipe to databases
- Wirelessly communicate via the MQTT protocol

## Dependencies
A physical interface is built on [PySerial](https://pyserial.readthedocs.io/en/latest/pyserial.html) or [pigpio](https://abyz.me.uk/rpi/pigpio/), in which newly-defined classes and objects are created to communicate between a master (Linux system, or dev board like RPi) and slaves.
Several industrial IoT devices (or called slaves inside the project files) could be communicated, such as the Mass Flow Controllers, Dry Gas Meters, Scales, Pumps, Heaters, Blowers, Fans, PWM-driven modules, RS485 / RS232 modules, and more.

In order to not block any subprocess while executing the main thread and maximize the python computing power out of the multi-core chip, the framework applies [multiprocessing](https://docs.python.org/3/library/multiprocessing.html).
However, for much easier multitasking, one would not need to apply multiprocessing or threading.
> Reminders: 
Let's say, you have a handful of processes going on in your project.
I'll recommend you to go see the python multiprocessing document, where multitasking could be realized with the help of many cores.   
Instead of applying the python threading package, which is another way to handle multitasking projects through a concept of context-switching with the help of a single core.

Analyzed Data are then piped to a remote database. For instance, [python mariadb](https://pypi.org/project/mariadb/). 
The data are processed further in another [visualization project](https://github.com/PeterTsungYu/dev_eda) to generate reports.

Simultaneously, those data can be delivered to other platforms via [phao MQTT](https://pypi.org/project/paho-mqtt/). This is a wireless communication protocol between subscribers, publishers, and a server.
Data conveys either from the master as a publisher or to the master as a subscriber.

With this IoT project folder, one could further pipe data to a [NodeRed platform](https://nodered.org/) for monitoring, controlling, and visualizing. 
Take a look at this [NodeRed dashboard](https://github.com/PeterTsungYu/flows_Reformer) that is created under the same scope of this project.   

| Features              | Links                   |
| -----------------     |:----------------------- |
| NodeRed dashboard     | [:link:][https://github.com/PeterTsungYu/flows_Reformer]    |
| Visualization project | [:link:][https://github.com/PeterTsungYu/dev_eda]           |

## Quick Start
Below instructions are employed in the environment as...
- Linux-based or Linux system
- Raspberry Pi board (Rpi 3B+ is used in this project)
- Available USB ports (4 ports on Rpi 3B+)
- Wifi or Intranet access

### Clone the project
```shell
git clone https://github.com/PeterTsungYu/dev_iot.git
```

### Install required packages
```shell
cd dev_iot/
pip install -r requirements.txt
```

### Install MQTT Eclipse
```shell
sudo apt update
sudo apt install mosquitto mosquitto-clients

# to start the service
sudo systemctl start mosquitto

# to stop the service 
sudo systemctl stop mosquitto

# to enable the service on boot (/lib/systemd)
sudo systemctl enable mosquitto

# to disable the service on boot (/lib/systemd)
sudo systemctl disable mosquitto
```

### MariaDB as mySQL drop-in replacement
> If you wish to apply a pipe to a database, you should establish a database somewhere. Either as a localhost or as a remoe database server.

```shell
# install MariaDB
sudo apt-get install mariadb-server

# for secure setting
sudo mysql_secure_installation

# check systemctl status 
systemctl status mysql

# to start the service
sudo systemctl start mysql

# to stop the service 
sudo systemctl stop mysql

# for login as root 
$ sudo mysql -u root -p
```

```
# inside the shell of mysql
## create a new db
CREATE DATABASE <example>;

## create a table in a db
create table <customer>(<name> varchar(10), <join_date> date) DEFAULT CHARSET=utf8;

## create a new user
CREATE USER <user>@<IP> IDENTIFIED by <password>;

## grant privilege
GRANT ALL PRIVILEGES ON <database>.<table> TO <user>@<IP>;
FLUSH PRIVILEGES;

## show grants
SHOW GRANTS FOR <user>@<IP>;
```

### .env file
You can find an example of an .env file inside the folder of .env_example.
```shell
touch .env 
```

```
# inside the .env, fill in the info
db_user=''
db_pwd=''
host_vpn=''
```
> "db_user", the one you created within mysql

> "db_pwd", the one you created within mysql as the user is created

> "host_vpn", it is the IP v4 address of your host of database


### Assign names to the USB port on RPi


### Create your slaves


### Define your topics to be published / subscribed


### Execute Python script in shell
```shell
python3 Threads_rasp.py
```
#### If start successfully
![dev_iot_img_0](https://i.imgur.com/K3yjgn5.png)

#### If analyze successfully
![dev_iot_img_1](https://i.imgur.com/0aL0qFd.png)

#### If shut down gracefully
![dev_iot_img_2](https://i.imgur.com/PfAyyde.png)




