#!/bin/bash

if [ ! $SUDO_PASSWORD ]
then
	echo "::::: Enter sudo password :::::"
	read -s SUDO_PASSWORD
fi

git clone http://student@192.168.0.203:17990/scm/~n.zakhvataev/minini.git

cd minini
mkdir build
cd build
cmake ..
make -j4
echo $SUDO_PASSWORD | sudo -S make install
