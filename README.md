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
The below instructions are employed in the environment as...
- Linux-based or Linux system with the Python3 installed 
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
You can find an example of an .env file inside [the folder of .env_example](https://github.com/PeterTsungYu/dev_iot/tree/6698671901cd63ed7120461a32db6666de7aecc7/.env_example).
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
```shell
# list all usb devices and their symlinks
ls -l /dev/ttyUSB*

#display the name of USB and its port name 
dmesg | grep ttyUSB
```
> You might find a usb port is indicated by "/dev/ttyUSB*".
Use this name as the flag below.  
The below example uses the name of /dev/ttyUSB0.

```shell
# display usb info
## look up for the name of USB. ex. KERNELS=="1-1.4"
## write down some unique identifiers
udevadm info --name=/dev/ttyUSB0 --attribute-walk
```
> Find attributes of the usb based on the section of KERNELs.
You might find a KERNELS, which it is assigned as "1-1.*".
For example, KERNELS=="1-1.4", without further suffixes.

> Once you locate the kernel, go ahead and paste the below snippet to the file named "10-usb-serial.rules".   
'''
KERNELS=="1-1.4", SUBSYSTEM=="tty", SYMLINK+="ttyUSB_Scale"
'''

```shell
# Creating the file with the USB port name rules
sudo nano /etc/udev/rules.d/10-usb-serial.rules

# Loading the new rules
sudo udevadm trigger
```
Once you have assigned a new name to your USB port. 
You can edit the [USB_port_name variables](https://github.com/PeterTsungYu/dev_iot/blob/6698671901cd63ed7120461a32db6666de7aecc7/config.py#L19) in the config.py file.

### Create your slaves
Stay in the config.py file. 
We are going to add in your slaves.
1. Give it an [slave's ID number](https://github.com/PeterTsungYu/dev_iot/blob/6698671901cd63ed7120461a32db6666de7aecc7/config.py#L27).
2. If it is a GPIO setting, assign [a gpio pin number](https://github.com/PeterTsungYu/dev_iot/blob/6698671901cd63ed7120461a32db6666de7aecc7/config.py#L49).
3. Edit your [slave_instances](https://github.com/PeterTsungYu/dev_iot/blob/6698671901cd63ed7120461a32db6666de7aecc7/config.py#L211).
4. Place your slaves to [corresponding ports](https://github.com/PeterTsungYu/dev_iot/blob/6698671901cd63ed7120461a32db6666de7aecc7/config.py#L627).
> For RPi, each slave is connected and wired to a specific USB port.
If it is a RS485 communication device, it might be wired to a RS485-RS232 adaptor, and then connected to a RS232-USB adaptor.
If it is a RS232 communication device, it might be connected to a RS232-USB adaptor.
>> The above-mentioned devices are connected with a Rpi via USB ports at the end.
If it is a GPIO-interfaced device, it might be wired to the GPIO on the RPi.

5. Make sure that your port is connected correctly to the assigned port with the given name. 
Edit the baudrate and other parameters to conform to the device's setting. 

6. Lastly, place your defined ports inside a variable called [list_ports](https://github.com/PeterTsungYu/dev_iot/blob/6698671901cd63ed7120461a32db6666de7aecc7/config.py#L695).
If you don't want to apply one of the ports upon runtime, feel free to comment it out.

### Define your topics to be published / subscribed
Stay in the config.py file. 
We are going to edit the published and subscribed topics where you will locate them at [slave_instances](https://github.com/PeterTsungYu/dev_iot/blob/6698671901cd63ed7120461a32db6666de7aecc7/config.py#L211).
> Note: Assign specific variable names without duplicates. These variable names will also be used in the NodeRed dashboard project.

### Execute Python script in shell
With everything being employed, good luck for your ride!
```shell
python3 Threads_rasp.py
```
#### If start successfully
![dev_iot_img_0](https://i.imgur.com/K3yjgn5.png)

#### If analyze successfully
![dev_iot_img_1](https://i.imgur.com/0aL0qFd.png)

#### If shut down gracefully
![dev_iot_img_2](https://i.imgur.com/PfAyyde.png)
