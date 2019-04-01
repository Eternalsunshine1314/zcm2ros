#!/bin/bash

if [ ! $SUDO_PASSWORD ]
then
	echo "::::: Enter sudo password :::::"
	read -s SUDO_PASSWORD
fi

git clone git://git.code.sourceforge.net/p/geographiclib/code geographiclib

cd geographiclib
mkdir build
./configure
make -j4
echo $SUDO_PASSWORD | sudo -S  make install
