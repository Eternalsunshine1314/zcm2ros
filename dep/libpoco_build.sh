#!/bin/bash

if [ ! $SUDO_PASSWORD ]
then
	echo "::::: Enter sudo password :::::"
	read -s SUDO_PASSWORD
fi


git clone git@github.com:pocoproject/poco.git
cd poco
./configure
make -j4
echo $SUDO_PASSWORD | sudo -S make install

echo $SUDO_PASSWORD | sudo -S ln -s /usr/local/lib/libPocoFoundation.so /usr/lib/libPocoFoundation.so