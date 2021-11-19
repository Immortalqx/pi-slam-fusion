#ifndef MAP2DFUSION_H
#define MAP2DFUSION_H

#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

#include <base/Svar/Svar.h>
#include <base/Svar/VecParament.h>
#include <base/time/Global_Timer.h>
#include "MainWindow.h"

#include "Map2D.h"

namespace Map2DFusion
{
    class TrajectoryLengthCalculator
    {
    public:
        TrajectoryLengthCalculator() : length(-1)
        {
        }

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

        virtual bool KeyPressHandle(void *arg)
        {
            QKeyEvent *e = (QKeyEvent *) arg;
            switch (e->key())
            {
                case Qt::Key_I:
                {
                    std::pair<cv::Mat, pi::SE3d> frame;
                    if (obtainFrame(frame))
                    {
                        pi::timer.enter("Map2D::feed");
                        map->feed(frame.first, frame.second);
                        if (mainwindow.get() && tictac.Tac() > 0.033)
                        {
                            tictac.Tic();
                            mainwindow->update();
                        }
                        pi::timer.leave("Map2D::feed");
                    }
                }
                    break;
                case Qt::Key_P:
                {
                    int &pause = svar.GetInt("Pause");
                    pause = !pause;
                }
                    break;
                case Qt::Key_Escape:
                {
                    stop();
                    return false;
                }
                    break;
                default:
                    return false;
                    break;
            }
            return false;
        }

        int TestMap2DItem();

        bool obtainFrame(std::pair<cv::Mat, pi::SE3d> &frame);

        int testMap2D();

        virtual void run()
        {
            std::string act = svar.GetString("Act", "Default");
            if (act == "TestMap2DItem") TestMap2DItem();
            else if (act == "TestMap2D" || act == "Default") testMap2D();
            else std::cout << "No act " << act << "!\n";
        }

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