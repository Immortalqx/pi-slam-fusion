# Install OpenCV 2.4


## 1. Requirements:

### 1.1 FFMPEG
* for Ubuntu 18.04 (Linux Mint 19) 
```
sudo apt-get install libavcodec-dev libavdevice-dev libavfilter-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev
```

* for Ubuntu 16.04 (Linux mint 16)
```
sudo apt-get install libavcodec54 libavcodec-dev libavdevice53 libavdevice-dev libavfilter3 libavfilter-dev libavformat54 libavformat-dev libavutil-dev libavutil52 libswscale-dev libswscale2 libavresample-dev
```


### 1.2 Building tools

* Ubuntu 18.04 (LinuxMint 19):
```
    sudo apt-get install build-essential
    sudo apt-get install git cmake cmake-gui gcc-4.8 g++-4.8
    sudo apt-get install libgtk2.0-dev libsndio-dev libasound2-dev libsdl1.2-dev libjpeg-dev libpng-dev libtiff-dev
    sudo apt-get install libdc1394-22-dev libv4l-dev 
```


* Ubuntu 16.04 (Linux mint 16):
```
    sudo apt-get install build-essential
    sudo apt-get install git cmake cmake-gui
    sudo apt-get install gcc-4.6 g++-4.6
    sudo apt-get install libgtk2.0-dev libgtkglext1 libgtkglext1-dev
    sudo apt-get install libgstreamer1.0-dev libdc1394-22-dev libv4l-dev libjpeg-dev libpng-dev libtiff-dev
```


## 2. Build & Install

* download at: https://gitee.com/pi-lab/resources/blob/master/libs/opencv-2.4.9.zip
* extract opencv source package:
```
unzip opencv-2.4.9.zip
cd opencv-2.4.9
mkdir build; cd build
```

* so lib (min, with system libjpeg, libpng version):
```
cmake   -DWITH_CUDA=off -DWITH_OPENCL=off -DWITH_OPENGL=off \
        -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/opt/opencv-2.4.9 \
        -DBUILD_JPEG=off -DBUILD_JASPER=off -DBUILD_OPENEXR=off \
        -DBUILD_PNG=off -DBUILD_TIFF=off -DBUILD_ZLIB=off \
        -DWITH_FFMPEG=off \
        -DCMAKE_CXX_FLAGS="-std=c++11" \
        -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.8 \
        -DCMAKE_C_COMPILER=/usr/bin/gcc-4.8 \
        ..
```

* build & install
```
make -j8
sudo make install
```

