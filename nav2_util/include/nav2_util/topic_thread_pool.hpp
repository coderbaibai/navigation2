#pragma once
#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <boost/thread/thread.hpp>
#include <deque>

namespace nav2_util{
class TopicThreadPool;
class WrappedThread{
public:
    WrappedThread(const TopicThreadPool* pool):pool(pool),stop_flag(false){}
    void start(){
        this->mtx.lock();
        stop_flag = false;
        this->mtx.unlock();
        handle = std::thread([this]{
            for(;;)//轮询
            {
                std::function<void()> task;
                {
                    //获取同步锁
                    std::unique_lock<std::mutex> lock(this->mtx);
                    //线程会一直阻塞，直到有新的task，或者是线程要退出
                    this->condition.wait(lock,
                    [this] { return this->stop_flag || !this->tasks.empty(); });
                    //线程退出
                    if(this->stop_flag && this->tasks.empty())
                        return;
                    //将task取出
                    task = std::move(this->tasks.front());
                    //队列中移除以及取出的task
                    this->tasks.pop();
                }
                //执行task,完了则进入下一次循环
                task();
            }
        });
    }
    void stop(){
        std::unique_lock<std::mutex> lock(this->mtx);
        stop_flag = true;
        this->condition.notify_all();
    }
    template<class F>
    void enqueue(F&& task){
        std::unique_lock<std::mutex> lock(this->mtx);
        this->tasks.push(std::forward<F>(task));
        if(tasks.size()>=queue_size){
            tasks.pop();
        }
        this->condition.notify_one();
    }
    ~WrappedThread(){
        stop();
        if(handle.joinable()) handle.join();
    }
private:
    const TopicThreadPool* pool;
    std::thread handle;
    std::queue<std::function<void()>> tasks;
    std::mutex mtx;
    std::condition_variable condition;
    bool stop_flag;
    static const int queue_size = 2; 
};

class TopicThreadPool
{
public:
    TopicThreadPool();
    void start();
    void add(const std::string& str_key);
    template<class F, class... Args>
    auto enqueue(std::string&str_key, F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>;
private:
    std::unordered_map<std::string,WrappedThread> thread_map;

    boost::shared_mutex stop_mutex;
    bool stop;
};
}

nav2_util::TopicThreadPool::TopicThreadPool(){
    boost::unique_lock<boost::shared_mutex> sl(stop_mutex);
    stop = true;
}
inline void nav2_util::TopicThreadPool::start(){
    boost::unique_lock<boost::shared_mutex> sl(stop_mutex);
    stop = false;
}
void nav2_util::TopicThreadPool::add(const std::string& str_key){
    boost::shared_lock<boost::shared_mutex> sl(stop_mutex);
    if(!this->stop){
        throw std::runtime_error("add when stop");
    }
    thread_map.emplace(str_key,this);
    thread_map.at(str_key).start();
}

// add new work item to the pool
// 将队列压入线程池,其中f是要执行的函数， args是多有的参数
template<class F, class... Args>
auto nav2_util::TopicThreadPool::enqueue(std::string&str_key, F&& f, Args&&... args)
-> std::future<typename std::result_of<F(Args...)>::type>
{
    boost::shared_lock<boost::shared_mutex> sl(stop_mutex);
    if(this->stop){
        throw std::runtime_error("enqueue on stopped ThreadPool");
    }
    //返回的结果的类型，当然可以根据实际的需要去掉这个(gcc4.7的c++11不支持)
    using return_type = typename std::result_of<F(Args...)>::type;
    //将函数handle与参数绑定
    auto task = std::make_shared< std::packaged_task<return_type()> >(
                    std::bind(std::forward<F>(f), std::forward<Args>(args)...)
                );

    //after finishing the task, then get result by res.get() (mainly used in the invoked function)
    std::future<return_type> res = task->get_future();
    if(this->thread_map.find(str_key)==this->thread_map.end()){
        throw std::runtime_error("str_key不存在");
    }

    this->thread_map.at(str_key).enqueue([task]()
    {
        (*task)();
    });
    return res;
}