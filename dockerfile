# FROM resin/rpi-raspbian:wheezy
# COPY qemu-arm-static /usr/bin/qemu-arm-static

FROM sedden/rpi-raspbian-qemu:wheezy

RUN apt-get update && apt-get install -y build-essential
RUN mkdir -p /home/au
WORKDIR /home/au
ADD . /home/au
