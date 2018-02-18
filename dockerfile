FROM resin/rpi-raspbian:wheezy
RUN apt-get update && sudo apt-get --yes --no-install-recommends install binfmt-support qemu-user-static
RUN apt-get update && sudo apt-get --yes --no-install-recommends install qemu-user-static
COPY ./qemu-arm-static /usr/bin/qemu-arm-static
RUN apt-get install -y build-essential
RUN apt-get install python
RUN mkdir -p /home/au
WORKDIR /home/au
ADD . /home/au
