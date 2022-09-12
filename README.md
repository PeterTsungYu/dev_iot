The aims of this IoT project are to establish a framework for the below...
- Collect and analyze data
- Remotely automatic control
- Pipe to databases
- Wirelessly communicate via the MQTT protocol

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

| Features          | Links                   |
| ----------------- |:----------------------- |
|NodeRed dashboard  | [:link:][https://github.com/PeterTsungYu/flows_Reformer]   |
| Browser Extension | [:link:][HackMD-it]     |
| Book Mode         | [:link:][Book-mode]     |
| Slide Mode        | [:link:][Slide-mode]    | 
| Share & Publish   | [:link:][Share-Publish] |



![permalink setting demo](https://imgur.com/cLKPwj7)

# Clone the project

