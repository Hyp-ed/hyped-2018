FROM resin/beaglebone-black-debian
RUN [ "cross-build-start" ]
RUN apt-get update && apt-get install -y build-essential python
RUN mkdir -p /home/au
WORKDIR /home/au
ADD . /home/au
RUN [ "cross-build-end" ]