#ifndef PISLAM_H
#define PISLAM_H

#include <QApplication>
#include "GSLAM/core/MapFusion.h"
#include "tests/gtest.h"
#include "MainWindow.h"
#include "StackTrace.h"

namespace pislam
{
    extern "C" {
    GSLAM::SLAMPtr createSLAMInstance();
    }

    void slamScommandHandle(void *ptr, std::string cmd, std::string para);

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

        virtual GSLAM::MapPtr getMap()
        {
            if (slam) return slam->getMap();
            else return GSLAM::MapPtr(NULL);
        }


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