#ifndef MAP2DFUSION_H
#define MAP2DFUSION_H

#include <base/time/Global_Timer.h>
#include "MainWindow.h"
#include "Map2D.h"

namespace Map2DFusion
{
    class TrajectoryLengthCalculator
    {
    public:
        TrajectoryLengthCalculator();

        ~TrajectoryLengthCalculator();

        void feed(pi::Point3d position);

    private:
        double length;
        pi::Point3d lastPosition;
    };


    class TestSystem : public pi::Thread, public pi::gl::EventHandle
    {
    public:
        TestSystem();

        ~TestSystem();

        virtual bool KeyPressHandle(void *arg);

        int TestMap2DItem();

        bool obtainFrame(std::pair<cv::Mat, pi::SE3d> &frame);

        int testMap2D();

        int Map2DWithSLAM();

        virtual void run();

        std::string datapath;
        pi::TicTac tictac;
        SPtr<MainWindow_Map2DFusion> mainwindow;
        SPtr<std::ifstream> in;
        SPtr<Map2D> map;
        SPtr<TrajectoryLengthCalculator> lengthCalculator;
    };

    int _main_map2dfusion(int argc, char **argv);
}
#endif // MAP2DFUSION_H