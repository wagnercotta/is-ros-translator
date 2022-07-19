# is-ros-translator

This is a service for Intelligent Spaces (IS) responsible for the integration of the programmable environment with the Robots Operating System ROS.

## Usage

### Broker

```shell
docker run -d --rm -p 5672:5672 -p 15672:15672 rabbitmq:3.7.6-management
```

### Robot

If not setup:
```shell
sudo apt install ros-noetic-rosbridge-server
sudo apt install ros-noetic-tf2-web-republisher
```
Then

```shell
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun tf2_web_republisher tf2_web_republisher
```

### More information at:

[Roslibpy](https://roslibpy.readthedocs.io/en/latest/reference/index.html#ros-setup) - API Reference | Setup | Connecting to ROS


### Docker Commands

```shell
docker build -t viniciusbaltoe/is-ros-translator:0.0.9 -f etc/docker/Dockerfile .
docker run -ti --rm --network=host -p 5672:5672  viniciusbaltoe/is-ros-translator:0.0.9 /bin/bash
```
