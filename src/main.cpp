//#include "../gui/pislam.h"
#include "../Map2DFusion/Map2DFusion.h"

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "param is not right" << std::endl;
        return 0;
    }
    else if (std::string(argv[1]) == "pislam")
    {
        std::cout << "pislam will start!" << std::endl;
//        return pislam::_main_pislam(argc, argv);
        return 0;
    }
    else if (std::string(argv[1]) == "map2dfusion")
    {
        std::cout << "map2dfusion will start!" << std::endl;
        return Map2DFusion::_main_map2dfusion(argc, argv);
//        return 0;
    }
    else
    {
        std::cout << "unknown type:\t" << argv[1] << std::endl;
        return 0;
    }
}
