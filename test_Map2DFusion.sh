#!/bin/bash

num=$#
test1="-t"
test2="--test"
normal1="-n"
normal2="--normal"

cd build/Map2DFusion

#运行程序
if [ $num == 0 ]; then
	./map2dfusion conf=/home/lqx/Lab/pi-slam-fusion/Map2DFusion/Default.cfg DataPath=/home/lqx/Lab/DataSet/phantom3-npu-origin
elif [ $num == 1 ]; then
	cmd=$1
	if [ $cmd == $test1 ] || [ $cmd == $test2 ]; then
		./map2dfusion conf=/home/lqx/Lab/pi-slam-fusion/Map2DFusion/Default.cfg DataPath=/home/lqx/Lab/DataSet/phantom3-npu-test
	elif [ $cmd == $normal1 ] || [ $cmd == $normal2 ]; then
		./map2dfusion conf=/home/lqx/Lab/pi-slam-fusion/Map2DFusion/Default.cfg DataPath=/home/lqx/Lab/DataSet/phantom3-npu-origin
	else
		echo "ERROR: unknown param: $cmd!"
	fi
elif (( $num >= 2 )); then
    echo "ERROR: too many arguments!"
fi

cd ../..
