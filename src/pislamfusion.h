#ifndef PI_SLAM_FUSION_PISLAMFUSION_H
#define PI_SLAM_FUSION_PISLAMFUSION_H

#include "gui/pislam.h"

namespace pislamfusion
{
    //TODO 梳理一下流程
    // Map2DFusion应该比较好改，就两个地方需要修改，plane与frame。
    //      目前的想法是：让pi-slam先计算好plane，再用这个参数启动map2dfusion，之后map2dfusion就只收发frame；
    // pi-slam会有点麻烦，可能要改到SLAM代码里面去，思路还不太清楚。
    //      第一步应该是用C++实现RANSAC算法
    // pi-slam里比较关键的地方：
    //      获取点云: MapperDemo.cpp:497:void  Mapper::createNewMapPointsBow()
    //      获取位姿: TrackerOpt.cpp:352:bool Tracker::track(const SPtr<MapFrame> &frame)
    //      但好像不是上面这两个地方，忘了。。。


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

    int _main_pislamfusion(int argc, char **argv);
}

#endif //PI_SLAM_FUSION_PISLAMFUSION_H
