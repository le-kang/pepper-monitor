#!/bin/bash

source /home/nao/.bash_profile

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR

#nohup roslaunch rosbridge_server rosbridge_websocket.launch &> /home/nao/monitor/ros_bridge.log &
#echo $! > /home/nao/monitor/ros_bridge.pid
nohup python start_web_server.py &> /home/nao/monitor/web_server.log &
echo $! > /home/nao/monitor/web_server.pid