#include "Data.h"

MainData::MainData()
{
    this->argc = 0;
    this->argv = nullptr;
}

MainData::MainData(int argc, char **argv)
{
    this->argc = argc;
    this->argv = argv;
}

int MainData::get_argc()
{
    return this->argc;
}

char **MainData::get_argv()
{
    return this->argv;
}