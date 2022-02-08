#include <iostream>
#include <thread>
#include <armadillo>

class Single
{

public:
    // 获取单实例对象
    static Single *GetInstance()
    {
        if (m_instance_ptr == nullptr)
        {
            m_instance_ptr = new Single;
        }
        return m_instance_ptr;
    }

    // 打印实例地址
    void Print()
    {
        std::cout << "我的内存地址是:" << this << std::endl;
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
    static Single *m_instance_ptr;

    // 禁止外部构造
    Single()
    {
        std::cout << "构造函数" << std::endl;
    }
};

Single *Single::m_instance_ptr = nullptr;

void test()
{
    //假设创建线程之前有一些其他工作要做
    sleep(1);

    Single *single = Single::GetInstance();

    single->Print();
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