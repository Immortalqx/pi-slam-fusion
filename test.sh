#!/bin/bash

num=$#

pislam="-pislam"
map2dfusion="-map2dfusion"
pislamfusion="-pislamfusion"

#加载OpenCV2.4.9
#export PKG_CONFIG_PATH=/usr/local/opencv_2.4.9/lib/pkgconfig
#export LD_LIBRARY_PATH=/usr/local/opencv_2.4.9/lib

#运行程序

cd build

if [ $num == 0 ]; then
	./pislamfusion pislamfusion
elif [ $num == 1 ]; then
	cmd=$1
	if [ $cmd == $pislam ]; then
		./pislamfusion pislam conf=/home/immortalqx/Lab/pi-slam/NWPU.cfg
	elif [ $cmd == $map2dfusion ]; then
		./pislamfusion map2dfusion conf=/home/immortalqx/Lab/Map2DFusion/Default.cfg DataPath=/home/immortalqx/Lab/DataSet/phantom3-npu-origin
	elif [ $cmd == $pislamfusion ]; then
		./pislamfusion pislamfusion
	else
		echo "ERROR: unknown param: $cmd!"
	fi
elif (( $num >= 2 )); then
    echo "ERROR: too many arguments!"
fi

cd ..
