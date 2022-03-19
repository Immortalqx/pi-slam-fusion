#include "../gui/StackTrace.h"
#include "../gui/pislam.h"
#include "../Map2DFusion/Map2DFusion.h"


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
