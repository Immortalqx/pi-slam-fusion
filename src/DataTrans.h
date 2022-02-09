#ifndef PI_SLAM_FUSION_DATATRANS_H
#define PI_SLAM_FUSION_DATATRANS_H

#include <list>
#include <mutex>
#include <condition_variable>

#define Trans DataTrans<std::pair<std::string, pi::SE3d>>::Instance()
#define Trans_Plane DataTrans<pi::SE3d>::Instance()

template<typename T>
class DataTrans
{
private:
    std::list<T> m_queue;//队列
    std::mutex m_mutex;//全局互斥锁
    std::condition_variable_any m_notEmpty;//全局条件变量（不为空）
    std::condition_variable_any m_notFull;//全局条件变量（不为满）
    int m_maxSize{};//队列最大容量

private:
    //队列为空
    bool isEmpty() const
    {
        return m_queue.empty();
    }

    //队列已满
    bool isFull() const
    {
        return m_queue.size() == m_maxSize;
    }

    //构造函数
    DataTrans()
    {
        this->m_maxSize = 30;
    }

public:

    // 获取单实例对象
    static DataTrans &Instance()
    {
        /**
         * 局部静态特性的方式实现单实例。
         * 静态局部变量只在当前函数内有效，其他函数无法访问。
         * 静态局部变量只在第一次被调用的时候初始化，也存储在静态存储区，生命周期从第一次被初始化起至程序结束止。
         */
        static DataTrans instance;
        return instance;
    }

    void product(const T &v)
    {
        std::unique_lock<std::mutex> locker(m_mutex);
        while (isFull())
        {
            //生产者等待"产品队列缓冲区不为满"这一条件发生.
            //m_notFull.wait(m_mutex);
            
            //如果满了就把前面的丢掉！
            m_queue.pop_front();
        }
        m_queue.push_back(v);
        locker.unlock();
        m_notEmpty.notify_one();
    }

    void consumption(T &v)
    {
        std::unique_lock<std::mutex> locker(m_mutex);
        while (isEmpty())
        {
            // 消费者等待"产品队列缓冲区不为空"这一条件发生.
            m_notEmpty.wait(m_mutex);
        }
        //在队列里面消费一个元素,同时通知队列不满这个信号量
        v = m_queue.front();
        m_queue.pop_front();
        locker.unlock();
        m_notFull.notify_one();
    }

};

#endif //PI_SLAM_FUSION_DATATRANS_H
