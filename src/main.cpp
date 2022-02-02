#include "../gui/StackTrace.h"
#include "../gui/pislam.h"
#include "../Map2DFusion/Map2DFusion.h"

//TODO 梳理一下流程
// Map2DFusion应该比较好改，就两个地方需要修改，plane与frame。
//      目前的想法是：让pi-slam先计算好plane，再用这个参数启动map2dfusion，之后map2dfusion就只收发frame；
// pi-slam会有点麻烦，可能要改到SLAM代码里面去，思路还不太清楚。
//      第一步应该是用C++实现RANSAC算法
// pi-slam里比较关键的地方：
//      获取点云: MapperDemo.cpp:497:void  Mapper::createNewMapPointsBow()
//      获取位姿: TrackerOpt.cpp:352:bool Tracker::track(const SPtr<MapFrame> &frame)
//      但好像不是上面这两个地方，忘了。。。

int main(int argc, char **argv)
{
    int ret = 0;

    // FIXME: this utils failed under new GCC compiler, you can find the problem
    // enable stack trace
    pi::dbg_stacktrace_setup();

    svar.ParseMain(argc, argv);

    std::string act = svar.GetString("Act", "SLAM");
    if ("SLAM" == act)
    {
        QApplication app(svar.i["argc"], (char **) svar.GetPointer("argv"));

        GSLAM::MainWindow mainwindow;
        mainwindow.show();

        // 启动SLAM线程
        pislam::SLAM_System slamSystem(&mainwindow);

        //启动map2dfusion线程
        Map2DFusion::TestSystem sys;
        sys.start();

        // get direct dataset
        for (int i = 1; i < argc; i++)
        {
            std::string arg = argv[i];
            if (arg.find('=') == std::string::npos && arg.front() != '-')
            {
                mainwindow.slotOpen(arg.c_str());
            }
        }
        ret = app.exec();
    }
    return ret;
}
