#!/bin/bash

if [ "$UID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

wget http://www.ftdichip.com/Drivers/D2XX/Linux/libftd2xx1.0.4.tar.gz

tar -xzvf libftd2xx1.0.4.tar.gz

ARCH=`uname -m`

echo "ARCHITECTUR INSTALLED: $ARCH"

cp libftd2xx1.0.4/build/$ARCH/libftd2xx.so.1.0.4 /usr/local/lib

mkdir -p /usr/local/include/ftd2xx

cp libftd2xx1.0.4/ftd2xx.h /usr/local/include/ftd2xx/
cp libftd2xx1.0.4/WinTypes.h /usr/local/include/ftd2xx/

unlink /usr/local/lib/libftd2xx.so
unlink /usr/lib/libftd2xx.so

ln -s /usr/local/lib/libftd2xx.so.1.0.4 /usr/local/lib/libftd2xx.so
ln -s /usr/local/lib/libftd2xx.so.1.0.4 /usr/lib/libftd2xx.so

echo "INSTALL DONE"


