#!/bin/bash

if [ ! $SUDO_PASSWORD ]
then
	echo "::::: Enter sudo password :::::"
	read -s SUDO_PASSWORD
fi

./zcm_build.sh
./geolib_build.sh
./minini_build.sh
./libpoco_build.sh
#./opencv_build.sh