#!/bin/bash

num=$#

default="-default"
test="-test"

#运行程序

cd build

if [ $num == 0 ]; then
	./pislamfusion conf=/home/immortalqx/Lab/pi-slam-fusion/Default.cfg
elif [ $num == 1 ]; then
	cmd=$1
	if [ $cmd == $default ]; then
		./pislamfusion pislam conf=/home/immortalqx/Lab/pi-slam-fusion/Default.cfg
	elif [ $cmd == $test ]; then
		./pislamfusion map2dfusion conf=/home/immortalqx/Lab/pi-slam-fusion/Test.cfg
	else
		echo "ERROR: unknown param: $cmd!"
	fi
elif (( $num >= 2 )); then
    echo "ERROR: too many arguments!"
fi

cd ..
