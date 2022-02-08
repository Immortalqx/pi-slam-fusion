#include <thread>
#include <iostream>
#include "DataTrans.h"

void function_1()
{
    int count = 300;
    while (count > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        Trans.product(count);
        std::cout << "product:\t" << count << std::endl;
        count--;
    }
}

void function_2()
{
    int data = 0;
    while (data != 1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        Trans.consumption(data);
        std::cout << data << "\tis consumed!" << std::endl;
    }
}

int main()
{
    std::thread t1(function_1);
    std::thread t2(function_2);
    t1.join();
    t2.join();
    return 0;
}