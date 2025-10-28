
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

template <typename T>
class ThreadSafeQueue
{
 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;

 public:
  void push(const T &value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(value);
    cond_.notify_one();
  }

  bool pop(T &value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [this] { return !queue_.empty(); });
    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  bool empty()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }
};

void worker(ThreadSafeQueue<int> &queue, int id)
{
  while (true)
  {
    int msg;
    if (queue.pop(msg))
    {
      std::cout << msg << "\n";
    }
  }
}

int main(int argc, char **argv)
{
  ThreadSafeQueue<int> queue;

  queue.push(1);
  queue.push(2);
  queue.push(3);
  queue.push(4);

  const int NUM_THREADS = 4;
  std::vector<std::thread> workers;

  // Start worker threads
  for (int i = 0; i < NUM_THREADS; ++i)
  {
    workers.emplace_back(worker, std::ref(queue), i);
  }

  // Optionally, join threads (if you use a termination condition)
  for (auto &t : workers)
  {
    t.join();
  }

  // Attempting to join again to demonstrate error handling
  try
  {
    for (auto &t : workers)
    {
      t.join();
    } // ❌ ERROR: std::system_error
  }
  catch (const std::system_error &e)
  {
    std::cerr << "Second join failed: " << e.what() << '\n';
  }

  return 0;
}