#include "pislamfusion.h"
#include <iostream>

#include "Data.h"
#include "../gui/pislam.h"
#include "../Map2DFusion/Map2DFusion.h"

namespace pislamfusion
{
    //TODO
    int _main_pislamfusion(int argc, char **argv)
    {
        std::cout << "pislamfusion is not finished!" << std::endl;
        return 0;
    }

//    int _main_pislamfusion(int argc, char **argv)
//    {
//        MainData data(argc, argv);
//
//        //在这里运行下面的语句，不能够把解析的参数传到线程中去。。。
//        svar.ParseMain(argc, argv);
//        std::cout << "svar test\n";
//        std::cout << svar.GetInt("Win3D.Enable", 0) << std::endl;
//        std::cout << svar.GetString("Map2D.DataPath", "") << std::endl;
//        std::cout << "svar test end\n";
//        return 0;
//
//        //首先创建map2dfusion的线程（pthread）
//        pthread_attr_t attr;
//        pthread_t map2dfusion;
//        pthread_attr_init(&attr);
//        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
//        pthread_create(&map2dfusion, &attr, Map2DFusion::_thread_map2dfusion, (void *) (&data));
//
//        //之后直接运行pi-slam（后面应该也会修改成线程的）
//        //FIXME 为什么在main.cpp运行的时候没有问题，仅仅是把它挪动到这里来就会出现问题？？？（比如显示dataset为空，然后又继续正常运行。）
//        pislam::_main_pislam(argc, argv);
//        //销毁线程（问题：会不会上面的运行了，线程就没有运行？）
//        pthread_join(map2dfusion, nullptr);
//        //终端有输入就会让程序停止运行
//        getchar();
//        getchar();
//        return 0;
//    }

}