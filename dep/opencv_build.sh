#!/bin/bash

git clone git@github.com:opencv/opencv.git

cd opencv
mkdir build
cd build
cmake -D CMAKE_INSTALL_PREFIX=$(pwd) ..
make -j4
make DESTDIR=$(pwd) install

