FROM resin/rpi-raspbian:wheezy
RUN apt-get install -y build-essential
RUN apt-get install python
RUN mkdir -p /home/au
WORKDIR /home/au
ADD . /home/au
