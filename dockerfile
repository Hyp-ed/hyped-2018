FROM resin/rpi-raspbian:wheezy
COPY ./qemu-arm-static /usr/bin/qemu-arm-static
RUN apt-get update && apt-get install -y build-essential
RUN apt-get install python
RUN mkdir -p /home/au
WORKDIR /home/au
ADD . /home/au
