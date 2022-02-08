#include <iostream>
#include <thread>
#include <armadillo>

//静态局部变量的懒汉单例（C++11线程安全）
///  内部静态变量的懒汉实现
class Single
{

public:
    // 获取单实例对象
    static Single &GetInstance()
    {
        /**
         * 局部静态特性的方式实现单实例。
         * 静态局部变量只在当前函数内有效，其他函数无法访问。
         * 静态局部变量只在第一次被调用的时候初始化，也存储在静态存储区，生命周期从第一次被初始化起至程序结束止。
         */
        static Single signal;
        return signal;
    }

    // 打印实例地址
    void Print()
    {
        std::cout << "我的实例内存地址是:" << this << std::endl;
    }

    ~Single()
    {
        std::cout << "析构函数" << std::endl;
    }

    // 禁止外部拷贝构造
    Single(const Single &signal) = delete;

    // 禁止外部赋值操作
    const Single &operator=(const Single &signal) = delete;

private:
    // 禁止外部构造
    Single()
    {
        std::cout << "构造函数" << std::endl;
    }
};

void test()
{
    //假设开始前有准备工作要进行
    sleep(1);

    Single &single = Single::GetInstance();

    single.Print();
}

int main()
{
    std::thread T[10];
    for (auto &i: T)
        i = std::thread(test);

    //下面两个循环二选一即可！
    //for (auto &i: T)
    //    i.detach();
    //getchar();

    for (auto &i: T)
        if (i.joinable())
            i.join();

    return 0;
}