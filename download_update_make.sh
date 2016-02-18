#!/bin/bash

#wget https://github.com/iklln6/ocast_pi/archive/master.zip ~/Desktop/master.zip
#unzip ~/Desktop/THUMB1GB/hello_pi/OCAST_heyworld/master.zip -d ~/Desktop/foo
#cp ~/Desktop/foo/ocast_pi-master/ocast_hello_world.c ~/Desktop/THUMB1GB/hello_pi/OCAST_heyworld/
#rm ~/Desktop/master.zip
#make
wget https://github.com/iklln6/ocast_pi/archive/master.zip ~/Desktop/master.zip
unzip ~/Desktop/THUMB1GB/hello_pi/OCAST_heyworld/master.zip -d ~/Desktop/foo
rm ~/Desktop/THUMB1GB/hello_pi/OCAST_heyworld/master.zip
cp ~/Desktop/foo/ocast_pi-master/ocast_hello_world.c ~/Desktop/THUMB1GB/hello_pi/OCAST_heyworld
make
#sudo ./OCAST_heyworld.bin
