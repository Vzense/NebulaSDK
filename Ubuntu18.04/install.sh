#!/bin/sh

# Check if user is root/running with sudo
if [ `whoami` != root ]; then
    echo Please run this script with sudo
    exit
fi

ORIG_PATH=`pwd`
cd `dirname $0`
SCRIPT_PATH=`pwd`
cd $ORIG_PATH

if [ "`uname -s`" != "Darwin" ]; then
    cp -d ${SCRIPT_PATH}/Thirdparty/opencv-3.4.1/lib/* /usr/lib/x86_64-linux-gnu/
fi
