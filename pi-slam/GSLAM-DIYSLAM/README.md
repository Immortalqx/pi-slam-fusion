# DIYSLAM

## 1. Introduction
DIYSLAM is a slam implementation based on GSLAM, which is well designed with small modules. DIYSLAM is very suitable for slam beginers to learn feature based slam system. With DIYSLAM everyone is able to implement his/her own FeatureDetector, Matcher, Initializer, Tracker, Mapper, Estimator, Optimizer, Relocalizer, LoopDetector and LoopCloser. Every module is self contained so that you don't have to worry it will affect other modules.

## 2. Compile and Run
### 2.1. Install GSLAM 2.3.0

```
git clone https://github.com/zdzhaoyong/GSLAM --branch 2.3.0
cd GSLAM
mkdir build;cd build;cmake ..
make -j4; sudo make install
```

Also we recommand user to compile and install optmizerPBA, which implemented bundle adjust very efficiently.

```
git clone https://github.com/zdzhaoyong/GSLAM_OptimizerPBA
cd GSLAM_OptimizerPBA; mkdir build
cd build; cmake ..;
make -j4; sudo make install
```

### 2.2. Build DIYSLAM Plugin

```
git clone http://192.168.1.3/zhaoyong/DIYSLAM
cd DIYSLAM
mkdir build;cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

### 2.3. Usage, run with gslam

```
cd build
gslam conf=../Default.cfg Dataset=<datasetfile>
```

### 2.4. Supported Datasets

1. DroneMapUnified: Dataset=/mnt/server0/users/zhaoyong/Dataset/NPU/DroneMap/phantom3-centralPark/phantom3-centralPark-unified/config.cfg
2. RTMapper: Dataset=/mnt/server0/users/zhaoyong/Dataset/NPU/RTMapper/mavic-library/mavic-library.rtm
3. KITTI: Dataset=/mnt/server0/users/zhaoyong/Dataset/KITTI/odomentry/color/00/mono.kitti
4. TUMMono: Dataset=/mnt/server0/users/zhaoyong/Dataset/TUM/Monocular/mono.tum

User defined datasets can also be supported, see more about GSLAM::Dataset.


## 3. Implement your own SLAM modules

### 3.1. Modules in DIYSLAM

DIYSLAM is implemented with several modules listed below:

| ModuleName | Functional |
| ---| ---|
| Dataset         | GSLAM Module, used to load frame sequences from online or offline datasets.|
| Map             | GSLAM Module, used to manage mappoints, mapframes and loopdetector. |
| Estimator       | GSLAM Module, Used to estimate Homography, Fundamental, Essential, Affine, ICP, PnP, PlaneFit and so on.|
| Optimizer       | GSLAM Module, Used to implement bundle, pose graph optimization and so on.|
| LoopDetector    | GSLAM Module, Used to detect loops from inserted mapframes.|
| FeatureDetector | Used to detect keypoints and descriptors.|
| Matcher         | Used to find match between keypoints.|
| Tracker         | Estimate the poses of sequences frames from given map, and insert keyframes to Mapper.|
| Mapper          | Handle KeyFrames to update current map.|
| LoopCloser      | Detect and close loops.|
| Initializer     | Used by tracker to create new map.|
| Relocalizer     | Deprecated. Used by tracker to relocate frames when losted.|

### 3.2. Steps to implement your own Module

1. Create your own folder such as "zhaoyong" in "src" folder.
```
mkdir src/zhaoyong
```

2. Create your own implementation of Map, FeatureDetector, Matcher, Initializer, Tracker, Mapper, Relocalizer, LoopDetector, LoopCloser.
```
touch src/zhaoyong/MapperZhaoyong.cpp
```

3. Enable your own implementation by edit configuration file "Default.cfg" or just set when run.
```
gslam conf=../Default.cfg Dataset=<datasetfile> Mapper=Zhaoyong
```

4. Please NEVER edit the existed files but only your own folder, if you think revise is needed, please contact Zhaoyong(zd5945@126.com).
