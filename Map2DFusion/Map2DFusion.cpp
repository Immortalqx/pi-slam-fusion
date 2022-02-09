#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <base/Svar/VecParament.h>
#include <base/Svar/Svar.h>
#include <GSLAM/core/Svar.h>

#include "Map2DFusion.h"

#include "../src/DataTrans.h"

using namespace std;

namespace Map2DFusion
{
    TrajectoryLengthCalculator::TrajectoryLengthCalculator() : length(-1)
    {
    }

    TrajectoryLengthCalculator::~TrajectoryLengthCalculator()
    {
        cout << "TrajectoryLength:" << length << endl;
    }

    void TrajectoryLengthCalculator::feed(pi::Point3d position)
    {
        if (length < 0)
        {
            length = 0;
            lastPosition = position;
        }
        else
        {
            length += (position - lastPosition).norm();
            lastPosition = position;
        }
    }

    TestSystem::TestSystem()
    {
        //从GSLAM的svar更新到PIL的svar（不过其实也可以在main中用两种svar解析命令含参数?）
        for (const auto &it: svar.get_data()) p_svar.insert(it.first, it.second);

        if (p_svar.GetInt("Win3D.Enable", 1))
        {
            mainwindow = SPtr<MainWindow_Map2DFusion>(new MainWindow_Map2DFusion(0));
        }

    }

    TestSystem::~TestSystem()
    {
        stop();
        while (this->isRunning()) sleep(10);
        if (map.get())
            map->save(p_svar.GetString("Map.File2Save", "result.png"));
        map = SPtr<Map2D>();
        mainwindow = SPtr<MainWindow_Map2DFusion>();
    }

    bool TestSystem::KeyPressHandle(void *arg)
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
                int &pause = p_svar.GetInt("Pause");
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

    int TestSystem::TestMap2DItem()
    {
        cv::Mat img = cv::imread(p_svar.GetString("TestMap2DItem.Image", "data/test.png"));
        if (img.empty() || !mainwindow.get())
        {
            cerr << "No image or mainwindow found.!\n";
            return -1;
        }
//        cv::imshow("img",img);
        pi::SvarWithType<cv::Mat>::instance()["LastTexMat"] = img;

        mainwindow->getWin3D()->SetEventHandle(this);
        mainwindow->getWin3D()->setSceneRadius(1000);
        mainwindow->call("show");
        mainwindow->call("MapWidget" + p_svar.GetString(" TestMap2DItem.cmd",
                                                        " Map2DUpdate LastTexMat 34.257287 108.888931 0 34.253234419307354 108.89463874078366 0"));
    }

//    bool TestSystem::obtainFrame(std::pair<cv::Mat, pi::SE3d> &frame)
//    {
//        string line;
//        if (!getline(*in, line)) return false;
//        stringstream ifs(line);
//        string imgfile;
//        ifs >> imgfile;
//        imgfile = datapath + "/rgb/" + imgfile + ".jpg";
//        pi::timer.enter("obtainFrame");
//        frame.first = cv::imread(imgfile);
//        pi::timer.leave("obtainFrame");
//        if (frame.first.empty()) return false;
//        ifs >> frame.second;
//        if (p_svar.exist("GPS.Origin"))
//        {
//            if (!lengthCalculator.get())
//                lengthCalculator = SPtr<TrajectoryLengthCalculator>(
//                        new TrajectoryLengthCalculator());
//            lengthCalculator->feed(frame.second.get_translation());
//        }
//        return true;
//    }

    //TODO 这里后面要改成完全从pi-slam获取
    bool TestSystem::obtainFrame(std::pair<cv::Mat, pi::SE3d> &frame)
    {
        std::pair<std::string, pi::SE3d> trans_frame;
        Trans.consumption(trans_frame);

        string imgfile = trans_frame.first;
        imgfile = datapath + "/rgb/" + imgfile + ".jpg";
        pi::timer.enter("obtainFrame");
        frame.first = cv::imread(imgfile);
        pi::timer.leave("obtainFrame");
        if (frame.first.empty()) return false;
        frame.second = trans_frame.second;
        if (p_svar.exist("GPS.Origin"))
        {
            if (!lengthCalculator.get())
                lengthCalculator = SPtr<TrajectoryLengthCalculator>(
                        new TrajectoryLengthCalculator());
            lengthCalculator->feed(frame.second.get_translation());
        }
        return true;
    }

    int TestSystem::testMap2D()
    {
        cout << "Act=TestMap2D\n";
        //datapath = svar.GetString("Map2D.DataPath", "/home/immortalqx/Lab/DataSet/phantom3-npu-origin");
        datapath = p_svar.GetString("Map2D.DataPath", "");
        if (!datapath.size())
        {
            cerr << "Map2D.DataPath is not seted!\n";
            return -1;
        }
        p_svar.ParseFile(datapath + "/config.cfg");
        if (!p_svar.exist("Plane"));
        {
//            cerr<<"Plane is not defined!\n";
//            return -2;
        }

        if (!in.get())
            in = SPtr<ifstream>(new ifstream((datapath + "/trajectory.txt").c_str()));

        if (!in->is_open())
        {
            cerr << "Can't open file " << (datapath + "/trajectory.txt") << endl;
            return -3;
        }
        deque<std::pair<cv::Mat, pi::SE3d>> frames;
        for (int i = 0, iend = p_svar.GetInt("PrepareFrameNum", 10); i < iend; i++)
        {
            std::pair<cv::Mat, pi::SE3d> frame;
            if (!obtainFrame(frame)) break;
            frames.push_back(frame);
        }
        cout << "Loaded " << frames.size() << " frames.\n";

        if (!frames.size()) return -4;

        map = Map2D::create(p_svar.GetInt("Map2D.Type", Map2D::TypeGPU),
                            p_svar.GetInt("Map2D.Thread", true));
        if (!map.get())
        {
            cerr << "No map2d created!\n";
            return -5;
        }
        VecParament vecP = p_svar.get_var("Camera.Paraments", VecParament());
        if (vecP.size() != 6)
        {
            cerr << "Invalid camera parameters!\n";
            return -5;
        }

        //TODO 接受plane参数
        // 在这里需要传入plane的数据（pi-slam先用ransac计算出来，再想办法传这里来！）
        // 1. ransac算法的C++实现倒是还没有写，先把ransac算法写好！
        // 2. 怎么传过来呢？flag+plane，两个变量？
        map->prepare(p_svar.get_var<pi::SE3d>("Plane", pi::SE3d()),
                     PinHoleParameters(vecP[0], vecP[1], vecP[2], vecP[3], vecP[4], vecP[5]),
                     frames);

        if (mainwindow.get())
        {
            mainwindow->getWin3D()->SetEventHandle(this);
            mainwindow->getWin3D()->insert(map);
            mainwindow->getWin3D()->setSceneRadius(1000);
            mainwindow->call("show");

            if (!p_svar.exist("GPS.Origin")) p_svar.i["Fuse2Google"] = 0;
            else
                p_svar.ParseLine("SetCurrentPosition $(GPS.Origin)");
            tictac.Tic();
        }
        else
        {
            int &needStop = p_svar.GetInt("ShouldStop");
            while (!needStop) sleep(20);
        }

        if (p_svar.GetInt("AutoFeedFrames", 1))
        {
            pi::Rate rate(p_svar.GetInt("Video.fps", 100));
            //这里是最后被阻塞的地方
            while (!shouldStop())
            {
                if (map->queueSize() < 2)
                {
                    std::pair<cv::Mat, pi::SE3d> frame;
                    if (!obtainFrame(frame)) break;
                    map->feed(frame.first, frame.second);
                }
                if (mainwindow.get() && tictac.Tac() > 0.033)
                {
                    tictac.Tic();
                    mainwindow->getWin3D()->update();
                }
                rate.sleep();
            }
        }
    }

    void TestSystem::run()
    {
        std::string act = p_svar.GetString("Act", "Default");
        if (act == "TestMap2DItem") TestMap2DItem();
        else if (act == "TestMap2D" || act == "Default") testMap2D();
        else std::cout << "No act " << act << "!\n";
    }

    int _main_map2dfusion(int argc, char **argv)
    {
        p_svar.ParseMain(argc, argv);

        if (p_svar.GetInt("Win3D.Enable", 0))
        {
            QApplication app(argc, argv);
            TestSystem sys;
            sys.start();
            return app.exec();
        }
        else
        {
            TestSystem sys;
            sys.run();
        }
        return 0;
    }
}
