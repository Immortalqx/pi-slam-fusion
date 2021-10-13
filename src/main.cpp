#include <QApplication>

#include <algorithm>

#include "GSLAM/core/Svar.h"
#include "GSLAM/core/Timer.h"
#include "GSLAM/core/MapFusion.h"
#include "GSLAM/core/TileManager.h"

#ifdef  ENABLE_MEMORYCHECK
#include "GSLAM/core/MemoryMetric.inc"
#endif

#include "GSLAM/core/Utils.inc"


#include "pi-slam/tests/gtest.h"
#include "MainWindow.h"
#include "StackTrace.h"


using namespace std;
using namespace GSLAM;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

extern "C" {
GSLAM::SLAMPtr createSLAMInstance();
}

void slamScommandHandle(void *ptr, string cmd, string para);

class SLAM_System : public GSLAM::SLAM, public GObjectHandle
{
public:
    enum SlamStatus
    {
        STOP, RUNNING, PAUSE
    };

    SLAM_System(GSLAM::MainWindow *mw) : status(STOP)
    {
        // regist scommand
        scommand.RegisterCommand("SLAM_Call", slamScommandHandle, this);

#ifdef ENABLE_MEMORYCHECK
        GSLAM::MemoryMetric::instanceCPU().enable();
#endif

        // create SLAM instance
        slam = createSLAMInstance();
        slam->setCallback(dynamic_cast<GObjectHandle *>(this));
        slam->call("SetSvar", &svar);

        // get mainwindow & widgets
        mainWindow = mw;
        mainWindow->addSLAM(slam.get());

        frameVis = mw->getFrameVisualizer();
        slamVis = mw->getSLAMVisualizer();
        dataset = mw->getDataset();
    }

    ~SLAM_System()
    {
        stop();
        if (threadSLAM.joinable()) threadSLAM.join();
    }

    int start(void)
    {
        status = RUNNING;
        threadSLAM = std::thread(&SLAM_System::slamThread, this);

        return 0;
    }

    int pause(void)
    {
        status = PAUSE;
        return 0;
    }

    int stop(void)
    {
        status = STOP;
        return 0;
    }

    virtual bool setCallback(GObjectHandle *cbk)
    {
        _handle = cbk;
        return true;
    }

    virtual void handle(const SPtr<GObject> &obj)
    {
        if (!obj) return;

        if (SPtr<GSLAM::MapFrame> frame = std::dynamic_pointer_cast<GSLAM::MapFrame>(obj))// SLAM KeyFrame CallBack
        {
            if (!frame) return;

            frame->setImage(GSLAM::GImage());
        }

        // do visualization handle
        slamVis->handle(obj);
    }

    virtual bool valid() const
    {
        return true;
    }

    virtual void draw()
    {
        if (slam) slam->draw();
    }

    virtual bool isDrawable()
    {
        if (slam) return slam->isDrawable();
        else return false;
    }

    virtual MapPtr getMap()
    {
        if (slam) return slam->getMap();
        else return MapPtr(NULL);
    }


    void slamThread(void)
    {
        Rate rate(svar.GetInt("Frequency", 100));
        string str = slam->type() + "::Track";

        while (1)
        {
            if (status == STOP) break;

            rate.sleep();
            if (status == PAUSE)
            {
                continue;
            }

            // get a frame
            FramePtr Fr = dataset->grabFrame();
            if (!Fr) break;
            frameVis->setFrame(Fr);

            // track current frame
            timer.enter(str.c_str());
            slam->track(Fr);
            timer.leave(str.c_str());
        }

        // wait SLAM finish
        FramePtr fr;
        slam->track(fr);
        printf("SLAM finished!\n");


        // clear slam obj for visualization
        slamVis->setSLAM(NULL);
        slamVis->update();

        dataset->close();
        slam.reset();


#ifdef  ENABLE_MEMORYCHECK
        printf("dumpMemoryUsage - begin\n");
        MemoryMallocAnalysis mma;
        mma.dumpMemoryUsage("memusage_bycount.txt", MemoryMallocAnalysis::SORT_CALLCOUNT);
        mma.dumpMemoryUsage("memusage_bysize.txt", MemoryMallocAnalysis::SORT_MEMSIZE);
        printf("dumpMemoryUsage - end\n");
#endif

        // set stop flag & stop GUI
        status = STOP;
        mainWindow->slotStop();
    }

public:
    SlamStatus status;
    std::thread threadSLAM;

    GSLAM::SLAMPtr slam;
    GSLAM::MapFusionPtr mapFusion;

    GSLAM::DatasetPtr dataset;   // current dataset, load implementations

    GSLAM::MainWindow *mainWindow;
    GSLAM::FrameVisualizer *frameVis;  // image viewer
    GSLAM::SLAMVisualizer *slamVis;   // SLAM visualizer
};


void slamScommandHandle(void *ptr, string cmd, string para)
{
    //LOG(INFO) << "cmd: " << cmd << ", param: " << para;

    if (cmd == "SLAM_Call")
    {
        SLAM_System *s = (SLAM_System *) ptr;
        if (para == "Start")
            s->start();
        else if (para == "Pause")
            s->pause();
        else if (para == "Stop")
            s->stop();

        return;
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    int ret = 0;

    // enable stack trace
    pi::dbg_stacktrace_setup();

    // memory check
#ifdef ENABLE_MEMORYCHECK
    first_backtrace();
#endif

    // fix the possible problem of measuring time
    timer.disable();

    timer.enter("Main");
    svar.ParseMain(argc, argv);

    string act = svar.GetString("Act", "SLAM");
    if ("SLAM" == act)
    {
        QApplication app(svar.i["argc"], (char **) svar.GetPointer("argv"));

        GSLAM::MainWindow mainwindow;
        mainwindow.show();

        SLAM_System slamSystem(&mainwindow);

        // get direct dataset
        for (int i = 1; i < argc; i++)
        {
            string arg = argv[i];
            if (arg.find('=') == string::npos && arg.front() != '-')
            {
                mainwindow.slotOpen(arg.c_str());
            }
        }

        ret = app.exec();
    }
    else if ("Tests" == act)
    {
        testing::InitGoogleTest(&svar.i["argc"], (char **) svar.GetPointer("argv"));
        ret = RUN_ALL_TESTS();
    }

    timer.leave("Main");

    return ret;
}
