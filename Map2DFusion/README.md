# Map2DFusion
------------------------------------------------------------------------------

![](./map2dfusion.gif)

## Brief Introduction
This is an open-source implementation of paper:
Map2DFusion: Real-time Incremental UAV Image Mosaicing based on Monocular SLAM.

Website : http://zhaoyong.adv-ci.com/map2dfusion/

Video   : https://www.youtube.com/watch?v=-kSTDvGZ-YQ

PDF     : http://zhaoyong.adv-ci.com/Data/map2dfusion/map2dfusion.pdf   

If you use this project for research, please cite our paper:

```
@CONFERENCE{zhaoyong2016Map2DFusion, 
	author={S. {Bu} and Y. {Zhao} and G. {Wan} and Z. {Liu}}, 
	booktitle={2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
	title={Map2DFusion: Real-time incremental UAV image mosaicing based on monocular SLAM}, 
	year={2016}, 
	volume={}, 
	number={}, 
	pages={4564-4571}, 
	doi={10.1109/IROS.2016.7759672}, 
	ISSN={2153-0866}, 
	month={Oct}
}
```

## 1. Compilation
### 1.1. Resources
Download the latest code with: 
```
git clone git@gitee.com:pi-lab/Map2DFusion.git
```

### 1.2. Dependencies

建议使用Ubuntu 18.04 
```
sudo apt-get install build-essential g++ cmake git
sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
sudo apt-get install libqt4-dev libqt4-opengl-dev
sudo apt-get install gcc-4.8 g++-4.8
```

- Warnning: Compilation with CUDA can be enabled after CUDA_PATH defined.

### 1.3 Install OpenCV
由于本项目使用老版本的Feature，因此必须使用OpenCV 2.4。需要确保系统里没有安装其他版本的OpenCV。具体安装的方法请参考[《OpenCV安装说明》](Thirdpart/opencv_install.md)


### 1.3. Compilation
```
mkdir build; cd build

cmake -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.8 \
      -DCMAKE_C_COMPILER=/usr/bin/gcc-4.8 \
      -DOpenCV_DIR=/opt/opencv-2.4.9/share/OpenCV \
      ..

make
```

## 2. Usage
Obtain the sample sequence and launch:

```
    git clone https://gitee.com/pi-lab/phantom3-village-kfs-master
    ./map2dfusion conf=../Default.cfg DataPath=phantom3-village-kfs
```

More sequences can be downloaded at the [NPU DroneMap Dataset](http://zhaoyong.adv-ci.com/npu-dronemap-dataset).



## 3. Contact

If you have any issue compiling/running Map2DFusion or you would like to know anything about the code, please contact the authors:

     Yong Zhao -> zd5945@126.com



