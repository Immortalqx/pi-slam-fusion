#include "pislamfusion.h"
#include <iostream>

#include "Data.h"
#include "../gui/pislam.h"
#include "../Map2DFusion/Map2DFusion.h"

namespace pislamfusion
{
//    //TODO
//    int _main_pislamfusion(int argc, char **argv)
//    {
//        std::cout << "pislamfusion is not finished!" << std::endl;
//        return 0;
//    }
    int _main_pislamfusion(int argc, char **argv)
    {
        MainData data(argc, argv);

        //首先创建map2dfusion的线程（pthread）
        pthread_attr_t attr;
        pthread_t map2dfusion;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
        pthread_create(&map2dfusion, &attr, Map2DFusion::_thread_map2dfusion, (void *) (&data));

        //之后直接运行pi-slam（后面应该也会修改成线程的）
        //pislam::_main_pislam(argc, argv);

        //销毁线程（问题：会不会上面的运行了，线程就没有运行？）
        //FIXME 这里出问题了，但大概率是线程运行的过程中出问题了。。。。。。
        pthread_join(map2dfusion, nullptr);
        //终端有输入就会让程序停止运行
        getchar();
        getchar();

        return 0;
    }

}