#ifndef PISLAM_H
#define PISLAM_H

#include "GSLAM/core/MapFusion.h"
#include "MainWindow.h"

namespace pislam
{
    class SLAM_System : public GSLAM::SLAM, public GSLAM::GObjectHandle
    {
    public:
        enum SlamStatus
        {
            STOP, RUNNING, PAUSE
        };

        SLAM_System(GSLAM::MainWindow *mw);

        ~SLAM_System();

        int start(void);

        int pause(void);

        int stop(void);

        virtual bool setCallback(GObjectHandle *cbk);

        virtual void handle(const SPtr<GObject> &obj);

        virtual bool valid() const;

        virtual void draw();

        virtual bool isDrawable();

        virtual GSLAM::MapPtr getMap();

        void slamThread(void);

    public:
        SlamStatus status;
        std::thread threadSLAM;

        GSLAM::SLAMPtr slam;
        GSLAM::MapFusionPtr mapFusion;

        GSLAM::DatasetPtr dataset;   // current dataset, load implementations

        GSLAM::MainWindow *mainWindow;
        GSLAM::FrameVisualizer_local *frameVis;  // image viewer
        GSLAM::SLAMVisualizer *slamVis;   // SLAM visualizer
    };


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

    int _main_pislam(int argc, char **argv);
}

#endif // PISLAM_H