#include <iostream>
#include <thread>
#include <stack>

using namespace std;

//两个全局变量
bool flag = true;
stack<int> frame;

void func_1();

void func_2(int plane);


int main()
{
    //这里是主线程
    cout << "主线程启动！" << endl;

    //启动定位线程
    thread T_1(func_1);
    T_1.detach();

    //终端有输入就会让程序停止运行
    getchar();
    flag = false;
    if (T_1.joinable())
        T_1.join();
    return 0;
}

void func_1()
{
    //这里是子线程1
    // TODO
    //  假设这里是被启动的定位线程，需要做的有三件事情：
    //  1. 最基本的，实现定位的功能
    //  2. 计算一次平面的参数，通过这个参数启动建图线程
    //  3. 不断地给建图线程发送定位数据和图像数据

    cout << "定位线程启动！" << endl;

    double pose = 0.0;

    while (flag)
    {
        cout << "Current Pose:\t" << pose << endl;

        if (pose == 1.00)
        {
            int plane = 0x1010;
            thread T_2(func_2, plane);
            T_2.detach();
        }

        pose += 0.25;

        if (frame.size() > 20)
        {
            frame.pop();
            cout << "有未处理的帧被丢弃" << endl;
        }

        frame.push(int(pose * 4));
        this_thread::sleep_for(chrono::milliseconds(300));
    }
}

void func_2(int plane)
{
    //这里是子线程2
    // TODO
    //  假设这里是建图线程，需要做的有两件事情：
    //  1. 根据给定的平面参数启动本线程
    //  2. 不断地接收定位数据和图像数据，并且根据定位数据实现建图

    cout << "建图线程启动！" << endl;
    cout << "Plane:\t" << plane << endl;

    while (flag)
    {
        //TODO 这里需要想办法做一个通信！先尝试一下全局变量
        if (!frame.empty())
        {
            int f = frame.top();
            cout << "receive frame:\t" << f << endl;
            frame.pop();
        }
        this_thread::sleep_for(chrono::milliseconds(800));
    }
}
