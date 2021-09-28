// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: zd5945@126.com (Yong Zhao) 353184965@qq.com(Guochen Liu)
//
// Messenger: A light-weight, efficient, thread-safe message publish and
// subscribe tool similar with ROS, a popular robot operating system
// The tool has the following features:
// * Header only based on c++11, no extra dependency, makes it portable.
// * Thread safe and support multi-thread condition notify mode by setting the queue size.
// * Able to transfer any classes efficiently, including ROS defined messages, which means it can replace ROS messagging or work with it.

#ifndef GSLAM_MESSENGER_H
#define GSLAM_MESSENGER_H


#include <functional>
#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>

#ifdef __GNUC__
#include <cxxabi.h>
#endif

#include <atomic>
#include <condition_variable>
#include <future>
#include <queue>
#include <thread>

#include "Glog.h"

namespace GSLAM {

namespace detail{

// A simple threadpool implementation.
class ThreadPool {
public:
    // All the threads are created upon construction.
    explicit ThreadPool(const int num_threads) : stop(false) {
        for (size_t i = 0; i < num_threads; ++i) {
            workers.emplace_back([this] {
                for (;;) {
                    std::function<void()> task;

                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(
                                    lock, [this] { return this->stop || !this->tasks.empty(); });
                        if (this->stop && this->tasks.empty()) return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }

                    task();
                }
            });
        }
    }
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : workers) worker.join();
    }

    // Adds a task to the threadpool.
    template <class F, class... Args>
    auto Add(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>;

    size_t taskNumLeft() { return tasks.size(); }
    void   popTask(){
        std::unique_lock<std::mutex> lock(queue_mutex);
        tasks.pop();
    }
private:
    // Keep track of threads so we can join them
    std::vector<std::thread> workers;
    // The task queue
    std::queue<std::function<void()> > tasks;

    // Synchronization
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};

// add new work item to the pool
template <class F, class... Args>
auto ThreadPool::Add(F&& f, Args&&... args)
-> std::future<typename std::result_of<F(Args...)>::type> {
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared<std::packaged_task<return_type()> >(
                std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);

        // don't allow enqueueing after stopping the pool
        if (stop)
            std::cerr << "The ThreadPool object has been destroyed! Cannot add more "
                         "tasks to the ThreadPool!";

        tasks.emplace([task]() { (*task)(); });
    }
    condition.notify_one();
    return res;
}

static inline std::string demangle(const std::string &name)
{
#ifdef _MSC_VER
    // MSVC, then return name
    return name;
#elif defined(__GNUC__)
    // GCC, then use the code
    int status=0;
    char *p=abi::__cxa_demangle(name.c_str(), 0, 0, &status);
    std::string ret(p);
    free(p);
    return ret;
#else
    // show compiler error, when compiler is not supported
    #error unexpected c complier (msc/gcc), Need to implement this method for demangle
    return ""
#endif
}

} // end of namespace detail

class Messenger;
class Publisher;
class PubSubSpace;

class Subscriber {
public:
    typedef std::function<void(const std::shared_ptr<void>&)> CallBackFunc;

    Subscriber() {}
    Subscriber(const std::string& topic, const std::string& type,
               const CallBackFunc& callback, size_t queue_size = 0)
        : impl_(new Impl(topic,type,callback,queue_size)) {}
    ~Subscriber() {
        if(impl_.use_count()==2)
        {
            shutdown();
        }
    }

    /**
   * \brief Unsubscribe the callback associated with this Subscriber
   *
   * This method usually does not need to be explicitly called, as automatic
   * shutdown happens when
   * all copies of this Subscriber go out of scope
   *
   * This method overrides the automatic reference counted unsubscribe, and
   * immediately
   * unsubscribes the callback associated with this Subscriber
   */
    void shutdown() ;

    std::string getTopic() const {
        if (impl_) return impl_->topic_;
        return "";
    }

    std::string getTypeName() const {
        if (impl_) return impl_->type_;
        return "";
    }

    /**
   * \brief Returns the number of publishers this subscriber is connected to
   */
    uint32_t getNumPublishers() const ;

    operator void*() const { return (impl_) ? (void*)1 : (void*)0; }

    bool operator<(const Subscriber& rhs) const { return impl_ < rhs.impl_; }

    bool operator==(const Subscriber& rhs) const { return impl_ == rhs.impl_; }

    bool operator!=(const Subscriber& rhs) const { return impl_ != rhs.impl_; }

protected:
    friend class Messenger;
    friend class Publisher;
    struct Impl {
        Impl(const std::string& topic, const std::string& type,
             const CallBackFunc& callback, size_t queue_size = 0)
            : topic_(topic),
              type_(type),
              callback_(callback),
              unsubscribed_(false),
              queue_size_(queue_size),
              workthread_(queue_size ? new detail::ThreadPool(1) : nullptr) {}

        ~Impl(){workthread_.reset();}

        void publish(const std::type_info& typeinfo,
                     const std::shared_ptr<void>& message) const {
            if (unsubscribed_) return;
            if (workthread_ ){
                if(workthread_->taskNumLeft() >= queue_size_)
                    workthread_->popTask();
                workthread_->Add([this, message]() {
                    if (unsubscribed_) return;
                    callback_(message);
                });
                return;
            }
            callback_(message);
        }

        std::string topic_, type_;
        CallBackFunc callback_;
        bool unsubscribed_;
        std::shared_ptr<PubSubSpace> space_;
        size_t queue_size_;
        std::shared_ptr<detail::ThreadPool> workthread_;
    };

    Subscriber(std::shared_ptr<Subscriber::Impl> impl) : impl_(impl) {}

    virtual void publish(const std::type_info& typeinfo,
                         const std::shared_ptr<void>& message) const {
        if (!impl_) return;
        impl_->publish(typeinfo, message);
    }
    std::string key()const{return getTopic()+"#"+getTypeName();}

    std::shared_ptr<Impl> impl_;
};

class Publisher {
public:
    Publisher() {}
    Publisher(const std::string& topic, const std::string& type,
              size_t queue_size = 0)
        :impl_(new Impl(topic,type,queue_size)){}

    virtual ~Publisher() {
        if(impl_.use_count()==2)
        {
            shutdown();
        }
    }

    /**
   * \brief Publish a message on the topic associated with this Publisher.
   * The message should be copyable.
   */
    template <typename M>
    void publish(const M& message) const;

    /**
   * \brief Publish a message without a copy!
   */
    template <typename M>
    void publish(const std::shared_ptr<M>& message) const;

    /**
   * \brief Shutdown the advertisement associated with this Publisher
   *
   * This method usually does not need to be explicitly called, as automatic
   * shutdown happens when
   * all copies of this Publisher go out of scope
   *
   * This method overrides the automatic reference counted unadvertise, and does
   * so immediately.
   * \note Note that if multiple advertisements were made through
   * NodeHandle::advertise(), this will
   * only remove the one associated with this Publisher
   */
    void shutdown() ;

    /**
   * \brief Returns the topic that this Publisher will publish on.
   */
    std::string getTopic() const {
        if (impl_) return impl_->topic_;
        return "";
    }

    /**
   * \brief Returns the topic that this Publisher will publish on.
   */
    std::string getTypeName() const {
        if (impl_) return impl_->type_;
        return "";
    }

    /**
   * \brief Returns the number of subscribers that are currently connected to
   * this Publisher
   */
    uint32_t getNumSubscribers() const;

    /**
   * \brief Returns whether or not this topic is latched
   */
    bool isLatched() const { return getNumSubscribers(); }

    operator void*() const { return (impl_) ? (void*)1 : (void*)0; }

    bool operator<(const Publisher& rhs) const { return impl_ < rhs.impl_; }

    bool operator==(const Publisher& rhs) const { return impl_ == rhs.impl_; }

    bool operator!=(const Publisher& rhs) const { return impl_ != rhs.impl_; }


protected:
    friend class Messenger;
    friend class Subscriber;

    struct Impl {
        Impl(const std::string& topic, const std::string& type,
             size_t queue_size = 0)
            : topic_(topic),
              type_(type),
              queue_size_(queue_size),
              workthread_(queue_size ? new detail::ThreadPool(1) : nullptr) {}

        std::string topic_;
        std::string type_;
        std::shared_ptr<PubSubSpace> space_;
        size_t queue_size_;
        std::shared_ptr<detail::ThreadPool> workthread_;
        std::mutex mutex_;
    };
    Publisher(Impl* implement) : impl_(implement) {}
    std::string key()const{return getTopic()+"#"+getTypeName();}


    std::shared_ptr<Impl> impl_;
};

struct PubSubSpace
{
    std::mutex mtx_;
    std::set<Subscriber> subs_;
    std::set<Publisher>  pubs_;
};

class Messenger {
public:

    Messenger():d(new Data()){}
    virtual ~Messenger() {}

    static Messenger& instance(){
        static std::shared_ptr<Messenger> inst(new Messenger());
        return *inst;
    }

    template <class M>
    Publisher advertise(const std::string& topic, uint32_t queue_size = 0,
                        bool latch = false) {
        Publisher pub(topic, std::string(typeid(M).name()),queue_size);
        join(pub);
        return pub;
    }

    template <class M>
    Subscriber subscribe(
            const std::string& topic, uint32_t queue_size,
            std::function<void(const std::shared_ptr<M>&)> callback) {
        Subscriber sub(topic, typeid(M).name(),
                       *(Subscriber::CallBackFunc*)(&callback),
                       queue_size);
        join(sub);
        return sub;
    }

    template <class T, class M>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void (T::*fp)(const std::shared_ptr<M>&), T* obj) {
        std::function<void(const std::shared_ptr<M>&)> cbk =
                std::bind(fp, obj, std::placeholders::_1);
        return subscribe(topic, queue_size, cbk);
    }

    template <class M>
    Subscriber subscribe(const std::string& topic, int queue_size,
                         void (*fp)(const std::shared_ptr<M>&)) {
        return subscribe(topic, queue_size,
                         std::function<void(const std::shared_ptr<M>&)>(fp));
    }

    template <class M>
    Subscriber subscribe(const std::string& topic, int queue_size,
                         void (*fp)(const M&)){
        std::function<void(const std::shared_ptr<M>&)> cbk=
                [fp](const std::shared_ptr<M>& msg){
            fp(*msg);
        };
        return subscribe(topic,queue_size,cbk);
    }


    template <class T, class M>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void (T::*fp)(const M&), T* obj) {
        std::function<void(const std::shared_ptr<M>&)> cbk=
                [fp,obj](const std::shared_ptr<M>& msg){
            std::function<void(const M&)> call =
                    std::bind(fp, obj, std::placeholders::_1);
            call(*msg);
        };
        return subscribe(topic, queue_size, cbk);
    }

    std::vector<Publisher> getPublishers()const{
        std::unique_lock<std::mutex> lock(d->mutex_);
        std::vector<Publisher> pubs;
        for(auto it:d->spaces_)
            for(auto it1:it.second)
            {
                std::shared_ptr<PubSubSpace> space=it1.second;
                std::unique_lock<std::mutex> lock(space->mtx_);
                pubs.insert(pubs.end(),space->pubs_.begin(),space->pubs_.end());
            }
        return pubs;
    }

    std::vector<Subscriber> getSubscribers()const{
        std::unique_lock<std::mutex> lock(d->mutex_);
        std::vector<Subscriber> subs;
        for(auto it:d->spaces_)
            for(auto it1:it.second)
            {
                std::shared_ptr<PubSubSpace> space=it1.second;
                std::unique_lock<std::mutex> lock(space->mtx_);
                subs.insert(subs.end(),space->subs_.begin(),space->subs_.end());
            }
        return subs;
    }

    std::string introduction(int width=80)const{
        if(getPublishers().size()+getSubscribers().size()==0)
            return "";
        std::stringstream sst;
        sst<<"Publisher and Subscriber lists:\n";
        sst<<printTable({{width/5-1,"Type"},
                         {width*2/5-1,"Topic"},
                         {width*2/5,"Payload"}});

        for(int i=0;i<width;i++)
            sst<<"-";
        sst<<std::endl;

        for(const Publisher& pub:getPublishers()){
            sst<<printTable({{width/5-1,"Publisher"},
                             {width*2/5-1,pub.getTopic()},
                             {width*2/5,translate(pub.getTypeName())}});
        }

        for(const Subscriber& sub:getSubscribers()){
            sst<<printTable({{width/5-1,"Subscriber"},
                             {width*2/5-1,sub.getTopic()},
                             {width*2/5,translate(sub.getTypeName())}});
        }
        return sst.str();
    }

    static std::string printTable(std::vector<std::pair<int,std::string> > line){
        std::stringstream sst;
        while(true){
            size_t emptyCount=0;
            for(auto& it:line){
                size_t width=it.first;
                std::string& str=it.second;
                if(str.size()<=width){
                    sst<< std::setw(width)
                       <<std::setiosflags(std::ios::left)
                      <<str<<" ";
                    str.clear();
                    emptyCount++;
                }else{
                    sst<<str.substr(0,width)<<" ";
                    str=str.substr(width);
                }
            }
            sst<<std::endl;
            if(emptyCount==line.size()) break;
        }
        return sst.str();
    }

    static std::string translate(const std::string& name){
        static std::map<std::string, std::string> decode = {
            {typeid(int32_t).name(), "int32_t"},
            {typeid(int64_t).name(), "int64_t"},
            {typeid(uint32_t).name(), "uint32_t"},
            {typeid(uint64_t).name(), "uint64_t"},
            {typeid(u_char).name(), "u_char"},
            {typeid(char).name(), "char"},
            {typeid(float).name(), "float"},
            {typeid(double).name(), "double"},
            {typeid(std::string).name(), "string"},
            {typeid(bool).name(), "bool"},
        };
        auto it = decode.find(name);
        if (it != decode.end())
            return it->second;

        std::string result = GSLAM::detail::demangle(name);
        return result;
    }

    void join(const Publisher& pub){
        std::unique_lock<std::mutex> lock(d->mutex_);
        std::shared_ptr<PubSubSpace>& space=d->spaces_[pub.getTopic()][pub.getTypeName()];
        if(!space) space=std::shared_ptr<PubSubSpace>(new PubSubSpace());
        std::unique_lock<std::mutex> lock1(space->mtx_);
        space->pubs_.insert(pub);
        pub.impl_->space_=space;
    }

    void join(const Subscriber& sub){
        std::unique_lock<std::mutex> lock(d->mutex_);
        std::shared_ptr<PubSubSpace>& space=d->spaces_[sub.getTopic()][sub.getTypeName()];
        if(!space) space=std::shared_ptr<PubSubSpace>(new PubSubSpace());
        std::unique_lock<std::mutex> lock1(space->mtx_);
        space->subs_.insert(sub);
        sub.impl_->space_=space;
    }

    void join(Messenger another){
        for(const Publisher& pub:another.getPublishers()){
            join(pub);
        }
        for(const Subscriber& sub:another.getSubscribers()){
            join(sub);
        }
    }

private:
    struct Data{
        typedef std::map<std::string,std::shared_ptr<PubSubSpace>> type_spaces;
        std::mutex mutex_;
        std::map<std::string,type_spaces> spaces_;
    };
    std::shared_ptr<Data> d;
};

inline void Publisher::shutdown()
{
    if (!impl_) return;
    auto space=impl_->space_;
    std::unique_lock<std::mutex> lock(space->mtx_);
    auto it=space->pubs_.find(*this);
    impl_.reset();
    space->pubs_.erase(it);
}

inline uint32_t Publisher::getNumSubscribers() const {
    if (impl_) return impl_->space_->subs_.size();
    return 0;
}

inline uint32_t Subscriber::getNumPublishers() const {
    if (!impl_) return 0;
    return impl_->space_->pubs_.size();
}

inline void Subscriber::shutdown()
{
    if (!impl_) return;

    impl_->unsubscribed_ = true;
    auto space=impl_->space_;
    std::unique_lock<std::mutex> lock(space->mtx_);
    auto it=space->subs_.find(*this);
    impl_.reset();
    space->subs_.erase(it);
}

template <typename M>
void Publisher::publish(const std::shared_ptr<M>& message) const {
    if (!impl_) return;

    if (typeid(M).name() != impl_->type_) {
#ifdef HAS_GSLAM
        LOG(ERROR) << "Type mismatch:" << typeid(M).name() << " to "
                   << impl_->type_;
#else
        std::cerr << "Type mismatch:" << typeid(M).name() << " to " << impl_->type_
                  << "\n";
#endif
        return;
    }

    std::set<Subscriber> subscribers;
    {
        std::unique_lock<std::mutex> lock(impl_->space_->mtx_);
        subscribers = impl_->space_->subs_;
    }

    if (subscribers.empty()) {
        return;
    }

    if (impl_->workthread_ ) {

        if(impl_->workthread_->taskNumLeft() >= impl_->queue_size_)
            impl_->workthread_->popTask();
        impl_->workthread_->Add([subscribers, message]() {
            for (const Subscriber& s : subscribers) {
                s.publish(typeid(M), message);
            }
        });
        return;
    }

    for (Subscriber s : subscribers) {
        s.publish(typeid(M), *(const std::shared_ptr<int>*)&message);
    }
}

template <typename M>
void Publisher::publish(const M& m) const {
    M* message = new M(m);
    std::shared_ptr<M> msg(message);
    publish(msg);
}

}  // end of namespace GSLAM

#endif
