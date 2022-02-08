#include <iostream>
#include <thread>
#include <armadillo>
#include <memory> // shared_ptr
#include <mutex>  // mutex

class Single
{
public:
    typedef std::shared_ptr<Single> Ptr;

    // 获取单实例对象
    static Ptr GetInstance()
    {
        // "double checked lock"
        if (m_instance_ptr == nullptr)
        {
            std::lock_guard<std::mutex> lk(m_mutex);

            if (m_instance_ptr == nullptr)
            {
                m_instance_ptr = std::shared_ptr<Single>(new Single);
            }
        }
        return m_instance_ptr;
    }

    // 打印实例地址
    void Print()
    {
        std::cout << "我的内存地址是:" << this << std::endl;
    }

    //使用智能指针的时候不能够禁止外部析构！
    ~Single()
    {
        std::cout << "析构函数" << std::endl;
    }

    // 禁止外部拷贝构造
    Single(const Single &signal) = delete;

    // 禁止外部赋值操作
    const Single &operator=(const Single &signal) = delete;

private:
    static Ptr m_instance_ptr;
    static std::mutex m_mutex;

    // 禁止外部构造
    Single()
    {
        std::cout << "构造函数" << std::endl;
    }
};

// initialization static variables out of class
Single::Ptr Single::m_instance_ptr = nullptr;
std::mutex Single::m_mutex;

void test()
{
    //假设开始前有准备工作要进行
    sleep(1);

    Single::Ptr single = Single::GetInstance();

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