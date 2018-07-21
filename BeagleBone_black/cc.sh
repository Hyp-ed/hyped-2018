#!/bin/bash

# Cross-compile the BBB code on AWS
# This script uploads the src/ folder to an AWS instance, compiles the project and downloads the
# resulting executable

MAIN=""
TARGET=""
while getopts "hm:t:" opt; do
    case $opt in
        m)
            MAIN="MAIN=$OPTARG"
            ;;
        t)
            TARGET="$OPTARG"
            ;;
        *)
            echo "Usage: $0 [OPTIONS]"
            echo
            echo "Supported OPTIONS:"
            echo "  -h             Display this help message and exit"
            echo "  -m <main_file> Select a main to compile (e.g. '-m demo_bms.cpp')"
            echo "  -t <target>    Specify target (e.g. '-t all' or '-t lint')"
            exit 1
    esac
done

if [ ! -f .name.txt ]; then
    echo "No '.name.txt' file found; seems like this is the first run of the script."
    read -p 'Please enter a name: ' NAME
    echo $NAME > .name.txt
fi
NAME=$(cat .name.txt)
echo "Using name='$NAME'"

if ssh hyped@35.177.87.64 "[ -d /home/hyped/$NAME/hyped-2018 ]"; then
    echo "Remote directory found"
else
    printf "Creating remote directory and cloning the repo..."
    ssh hyped@35.177.87.64 "mkdir $NAME;\
                            cd ~/$NAME;\
                            git clone --quiet --recursive https://github.com/Hyp-ed/hyped-2018.git > /dev/null"
    printf "DONE\n"
fi

printf "Uploading src/ directory... "
scp -r src/ hyped@35.177.87.64:/home/hyped/$NAME/hyped-2018/BeagleBone_black > /dev/null
printf "DONE\n\n"

printf "Compiling...\n"
GPP="/home/hyped/gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++"
ssh -t hyped@35.177.87.64 "cd ~/$NAME/hyped-2018/BeagleBone_black;\
                        make clean;\
                        make -j $TARGET CC=$GPP LL=$GPP $MAIN"
printf "DONE\n\n"

printf "Downloading hyped executable...\n"
scp hyped@35.177.87.64:/home/hyped/$NAME/hyped-2018/BeagleBone_black/hyped .
