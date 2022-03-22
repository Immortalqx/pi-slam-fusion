#!/bin/bash

num=$#
fast1="-f"
fast2="--fast"
normal1="-n"
normal2="--normal"
help1="-h"
help2="--help"

normal_build()
{
	echo "normal build start!"
	
	rm -rf build
	mkdir build
	cd build
	
	cmake -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.8 \
		 -DCMAKE_C_COMPILER=/usr/bin/gcc-4.8 \
		 -DOpenCV_DIR=/usr/local/opencv_2.4.9/share/OpenCV \
		 ..

	#cmake -DOpenCV_DIR=/usr/local/opencv_2.4.9/share/OpenCV ..

	#flag1=#?
	
	make -j12
	#flag2=#?
	
	cd ..
	
	#if [ $flag1 == 0 ] && [ $flag2 == 0 ]; then
	#	echo "normal build succeed!"
	#else
	#	echo "ERROR: normal build failed!"
	#fi

	echo "normal build finished!"
}

fast_build()
{
	if [ ! -d "build" ]; then
		echo "WARNING: build folder does not exist, normal build will start！"
		normal_build
	else
		echo "fast build start"

		cd build

		make -j12
		#flag=#?
		
		cd ..
		
		#if [ $flag != 0 ]; then
		#	echo "fast build succeed"
		#else
		#    echo "ERROR: fast build failed!"
		#fi

		echo "fast build finished!"
	fi
}

run_help()
{
	echo "build.sh: 以指定的形式编译cmake工程"
	echo "usage: ./build.sh [选项]"
	echo "选项："
	echo "-h, --help		显示帮助信息"
	echo "-f, --fast		快速编译"
	echo "-n, --normal		正常编译"
}

if [ $num == 0 ]; then
    fast_build
elif [ $num == 1 ]; then
	cmd=$1
	if [ $cmd == $help1 ] || [ $cmd == $help2 ]; then
		run_help
	elif [ $cmd == $fast1 ] || [ $cmd == $fast2 ]; then
		fast_build
	elif [ $cmd == $normal1 ] || [ $cmd == $normal2 ]; then
		normal_build
	else
		echo "ERROR: unknown param: $cmd!"
	fi
elif (( $num >= 2 )); then
    echo "ERROR: too many arguments!"
fi


