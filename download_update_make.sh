#!/bin/bash

wget https://github.com/iklln6/ocast_pi/archive/master.zip ~/Desktop/master.zip
unzip ~/Desktop/master.zip -d ~/Desktop/foo
cp ~/Desktop/foo/ocast_hello_world.c ~/Desktop/THUMB1GB/hello_pi/OCAST_heyworld/
rm ~/Desktop/master.zip
make
