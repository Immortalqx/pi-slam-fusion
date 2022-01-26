#include "../gui/pislam.h"
#include "../Map2DFusion/Map2DFusion.h"
#include "pislamfusion.h"
#include "Data.h"

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "param is not right" << std::endl;
        return -1;
    }
    else if (std::string(argv[1]) == "pislam")
    {
        std::cout << "pislam will start!" << std::endl;
        return pislam::_main_pislam(argc, argv);
    }
    else if (std::string(argv[1]) == "map2dfusion")
    {
        std::cout << "map2dfusion will start!" << std::endl;
//        return Map2DFusion::_main_map2dfusion(argc, argv);
        //测试一下效果
        MainData data(argc, argv);
        Map2DFusion::_thread_map2dfusion((void *) (&data));
        return 0;
    }
    else if (std::string(argv[1]) == "pislamfusion")
    {
        //TODO 我觉得应该在这里把两个项目合起来！(或者pislamfusion类里面，然后尝试用类变量来通信？？？)
        std::cout << "pislamfusion will start!" << std::endl;
        return pislamfusion::_main_pislamfusion(argc, argv);
    }
    else
    {
        std::cout << "unknown type:\t" << argv[1] << std::endl;
        return -1;
    }
}
