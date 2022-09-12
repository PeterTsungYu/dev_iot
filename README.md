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
## restart, also start other dependent services
sudo systemctl start mosquitto

# to stop the service 
sudo systemctl stop mosquitto

# to enable the service on boot (/lib/systemd)
sudo systemctl enable mosquitto

# to disable the service on boot (/lib/systemd)
sudo systemctl disable mosquitto
```


