#!/bin/bash

echo ":::::   Build dependencies? (Y/n)  :::::"

read -r DEPS

echo ":::::     Enter sudo password     :::::"

read -s SUDO_PASSWORD

git submodule update --init --recursive


if [[ $DEPS == "Y"  ]] | [[ $DEPS == "y" ]]; then
	cd dep
	./build.sh
	cd ..
fi

src/zcm2ros/src/zcm_types/zcm_gen.sh

cd src/ibeo_core
cmake .
make -j4

cd ../zcm2ros
cmake .

echo $SUDO_PASSWORD | sudo -S chmod +x start.sh

cd ../..

catkin_make
