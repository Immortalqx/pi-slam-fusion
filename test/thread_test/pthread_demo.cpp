#include <pthread.h>
#include <iostream>
#include <stack>
#include <armadillo>
/*
 * 我发现pi-slam和map2dfusion用的都是封装过的pthread，所以也尝试了一下pthread的使用
 * 后面可能全改成std::thread，但看网上的资料说它跨平台做的比较好，功能上做的还是不如pthread
 */
/*
 *FIXME:
 * 1. 对互斥锁还是不太清楚，这个程序跑到一半就卡着不能运行了，暂时先没用互斥锁，这样程序还能跑
 * 2. pthread确实比std::thread难用一些，还要好好了解一下
 */
using namespace std;

//三个全局变量
bool flag = true;
int plane = -1;
stack<int> frame;

//互斥锁
pthread_mutex_t mutexsum;
//FIXME：忘了。。
pthread_attr_t attr;

void *func_1(void *pVoid)
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
//        pthread_mutex_lock(&mutexsum);

        cout << "Current Pose:\t" << pose << endl;

        if (pose == 1.00)
        {
            plane = 0x1010;
        }

        pose += 0.25;

        if (frame.size() > 20)
        {
            frame.pop();
            cout << "有未处理的帧被丢弃" << endl;
        }
        frame.push(int(pose * 4));

//        pthread_mutex_unlock(&mutexsum);
        sleep(1);
    }
    pthread_exit((void *) nullptr);
}

void *func_2(void *pVoid)
{
    //这里是子线程2
    // TODO
    //  假设这里是建图线程，需要做的有两件事情：
    //  1. 根据给定的平面参数启动本线程
    //  2. 不断地接收定位数据和图像数据，并且根据定位数据实现建图
    cout << "建图线程启动！" << endl;

    while (flag)
    {
//        pthread_mutex_lock(&mutexsum);
        if (plane != -1)
            break;
        cout << "wait for plane" << endl;

//        pthread_mutex_unlock(&mutexsum);
        sleep(2);
    }

    cout << "get Plane:\t" << plane << endl;
    while (flag)
    {
//        pthread_mutex_lock(&mutexsum);
        if (!frame.empty())
        {
            int f = frame.top();
            cout << "receive frame:\t" << f << endl;
            frame.pop();
        }
//        pthread_mutex_unlock(&mutexsum);
        sleep(1);
    }
    pthread_exit((void *) nullptr);
}

int main()
{
    //这里是主线程
    cout << "主线程启动！" << endl;

    //一些初始化步骤
    pthread_mutex_init(&mutexsum, nullptr);
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    //线程pi_slam
    pthread_t pi_slam;
    //线程map2dfusion
    pthread_t map2dfusion;
    pthread_create(&pi_slam, &attr, func_1, nullptr);
    pthread_create(&map2dfusion, &attr, func_2, nullptr);

    pthread_attr_destroy(&attr);

    void *status;
    pthread_join(pi_slam, &status);
    pthread_join(map2dfusion, &status);
    //终端有输入就会让程序停止运行
    getchar();
    getchar();
    flag = false;

    pthread_mutex_destroy(&mutexsum);
    pthread_exit(nullptr);
}
