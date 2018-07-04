#!/bin/bash

# Cross-compile the BBB code on AWS
# This script uploads the src/ folder to an AWS instance, compiles the project and downloads the
# resulting executable

getopts "m:" flag
MAIN=""
if [ $flag -eq "m" ]; then
    MAIN = "MAIN=$OPTARG"
fi
echo $MAIN

printf "Uploading src/ directory... "
scp -r src/ hyped@35.177.87.64:/home/hyped/Code/hyped-2018/BeagleBone_black > /dev/null
printf "DONE\n\n"

printf "Compiling...\n"
GPP="/home/hyped/gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++"
ssh hyped@35.177.87.64 "cd ~/Code/hyped-2018/BeagleBone_black;\
                        make clean;\
                        make -j CC=$GPP LL=$GPP $MAIN"
printf "DONE\n\n"

printf "Downloading hyped executable...\n"
scp hyped@35.177.87.64:/home/hyped/Code/hyped-2018/BeagleBone_black/hyped .
