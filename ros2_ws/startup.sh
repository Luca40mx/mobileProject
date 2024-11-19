#!/bin/bash

cd /workspace
chmod u+x ./*.sh
./build.sh
source ./install/setup.bash