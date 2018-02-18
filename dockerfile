FROM resin/rpi-raspbian:wheezy
RUN --rm --privileged multiarch/qemu-user-static:register --reset
RUN apt-get install -y build-essential
RUN apt-get install python
RUN mkdir -p /home/au
WORKDIR /home/au
ADD . /home/au
