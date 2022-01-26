#ifndef PI_SLAM_FUSION_DATA_H
#define PI_SLAM_FUSION_DATA_H

//用来传输数据的类
class Data
{
};

class MainData
{
public:
    MainData();

    MainData(int argc, char **argv);

    int get_argc();

    char **get_argv();

private:
    int argc;
    char **argv;
};


#endif //PI_SLAM_FUSION_DATA_H
