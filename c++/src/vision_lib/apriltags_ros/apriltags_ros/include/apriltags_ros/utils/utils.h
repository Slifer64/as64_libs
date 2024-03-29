#ifndef APRILTAGS_ROS_UTILS_H
#define APRILTAGS_ROS_UTILS_H

#include <mutex>
#include <condition_variable>
#include <thread>

namespace apriltags_ros
{

template<typename T>
class MtxVar
{
public:
  MtxVar() { }
  MtxVar& operator=(const T &val) { set(val); return *this; }
  bool operator()() const { return get(); }
  T get() const { std::unique_lock<std::mutex> lck(*(const_cast<std::mutex *>(&var_mtx))); return var; }
  T read() const { return var; }
  void set(const T &val) { std::unique_lock<std::mutex> lck(var_mtx); var=val; }
private:
  std::mutex var_mtx;
  T var;
};

class Semaphore
{
private:
  std::mutex mutex_;
  std::condition_variable condition_;
  // unsigned long count_ = 0; // Initialized as locked.
  bool count_ = false;  // Initialized as locked.

public:
  void notify()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    // ++count_;
    count_ = true;
    condition_.notify_one();
  }

  void wait()
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
      condition_.wait(lock);
    // --count_;
    count_ = false;
  }

  bool try_wait()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_)
    {
      // --count_;
      count_ = false;
      return true;
    }
    return false;
  }
};

} // namespace apriltags_ros

#endif // APRILTAGS_ROS_UTILS_H