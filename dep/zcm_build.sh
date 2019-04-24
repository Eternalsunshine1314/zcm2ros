#!/bin/bash

if [ ! $SUDO_PASSWORD ]
then
	echo "::::: Enter sudo password :::::"
	read -s SUDO_PASSWORD
fi

# git clone http://student@192.168.0.203:17990/scm/elsd/zcm.git
git clone https://github.com/ZeroCM/zcm
cd zcm

echo $SUDO_PASSWORD | sudo -S apt-get install -y libelf1 libelf-dev openjdk-8-jdk openjdk-8-jre nodejs nodejs-dev node-gyp npm node-gyp libssl1.0-dev python-pip cython
echo $SUDO_PASSWORD | sudo -S ln -s /usr/bin/nodejs /usr/bin/node


echo $SUDO_PASSWORD | sudo -S pip install python-config


## libzmq ##

echo $SUDO_PASSWORD | sudo -S apt-get install -y libtool pkg-config build-essential autoconf automake uuid-dev

git clone https://github.com/zeromq/libzmq
cd libzmq

mkdir build
cd build
cmake ..
make -j4
echo $SUDO_PASSWORD | sudo -S make install

cd ../..

echo "export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64/" >> ~/.bashrc
echo $SUDO_PASSWORD | sudo -S JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64/ ./waf configure --use-java --use-nodejs --use-python --use-zmq --use-elf --use-dev --use-inproc --use-ipc --use-udpm --use-serial
echo $SUDO_PASSWORD | sudo -S ./waf build 
echo $SUDO_PASSWORD | sudo -S ./waf install 

echo "export LD_LIBRARY_PATH=/usr/local/lib/:/usr/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc