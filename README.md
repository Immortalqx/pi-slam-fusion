# PI-SLAM-Fusion

## 1. 简介

基于PI-SLAM和Map2DFusion的无人机实时建图软件。

本系统将`GSLAM`，`DIYSLAM`，`Map2DFusion`整合在一起，并且为了降低编译的难度，将一些常用的第三方库整合在本项目中，因此仅仅依赖: `OpenCV`， `Qt`。为了方便学习和开发，最好在Linux下面来编写、编译、执行。



## 2. 编译

### 2.1 安装依赖（建议使用Ubuntu 18.04 ）：
```
sudo apt-get install build-essential g++ cmake git
sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
sudo apt-get install libqt4-dev libqt4-opengl-dev libpoco-dev
sudo apt-get install gcc-4.8 g++-4.8
```

### 2.2 安装OpenCV 2.4
由于本项目使用老版本的Feature，因此必须使用OpenCV 2.4。需要确保系统里没有安装其他版本的OpenCV。具体安装的方法请参考[《OpenCV安装说明》](thirdparty/opencv_install.md)


### 2.3 编译

```
mkdir build; cd build

cmake -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.8 \
      -DCMAKE_C_COMPILER=/usr/bin/gcc-4.8 \
      -DOpenCV_DIR=/opt/opencv-2.4.9/share/OpenCV \
      ..

make
```

如果不出意外，编译好的程序在cmake的编译目录，即`./build`目录下

经过上面的opencv安装教程，一般 `-DOpenCV_DIR=/opt/opencv-2.4.9/share/OpenCV` 可以直接使用，或者设置成`-DOpenCV_DIR=“安装了opencv-2.4.9的目录”`




## 3. 数据集
目前有两个数据集放在gitee上，具体的地址是：

* https://gitee.com/pi-lab/phantom3-npu

* https://gitee.com/pi-lab/phantom3-village-kfs-master


其他数据集包括：
1. `DroneMapUnified`: Dataset=/mnt/server0/users/zhaoyong/Dataset/NPU/DroneMap/phantom3-centralPark/phantom3-centralPark-unified/config.cfg
2. `RTMapper`: Dataset=/mnt/server0/users/zhaoyong/Dataset/NPU/RTMapper/mavic-library/mavic-library.rtm
3. `KITTI`: Dataset=/mnt/server0/users/zhaoyong/Dataset/KITTI/odomentry/color/00/mono.kitti
4. `TUMMono`: Dataset=/mnt/server0/users/zhaoyong/Dataset/TUM/Monocular/mono.tum

数据集的适配器代码放在`./gui/IO`目录下。可以参考上述的实现，通过继承`GSLAM::Dataset`来实现更多的数据集支持。


