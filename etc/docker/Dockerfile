FROM ros:noetic as build
ENV PATH="/opt/ros/noetic/bin:$PATH"
ENV PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages"

ARG folder
WORKDIR /opt/ros_translator_$folder

COPY ./requirements.txt .
COPY ./src/ros_translator_$folder .
COPY ./src/translate_functions ./translate_functions

RUN apt-get update
RUN apt install -y python3-pip
RUN apt install -y ros-noetic-std-msgs
RUN apt install -y ros-noetic-move-base 
RUN apt install -y ros-noetic-tf

RUN pip install -r requirements.txt
