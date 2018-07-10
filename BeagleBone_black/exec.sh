#!/bin/bash

# Upload hyped executable to BBB and run it

IP="192.168.7.2"
if [ "$1" != "" ]; then
    IP=$1
fi

scp hyped hyped@$IP:/home/hyped
ssh hyped@$IP "~/hyped"
