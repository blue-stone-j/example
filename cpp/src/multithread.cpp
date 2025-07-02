
#include <condition_variable>
#include <mutex>
#include <queue>

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

void worker(ThreadSafeQueue<rosbag::MessageInstance> &queue, int id)
{
  while (true)
  {
    rosbag::MessageInstance msg;
    if (queue.pop(msg))
    {
      // Example: handle sensor_msgs::Image
      sensor_msgs::Image::ConstPtr img = msg.instantiate<sensor_msgs::Image>();
      if (img != nullptr)
      {
        // process image
        std::cout << "Thread " << id << " processing image\n";
      }
    }
  }
}

void read_bag(ThreadSafeQueue<rosbag::MessageInstance> &queue)
{
  rosbag::Bag bag;
  bag.open("file.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics = {"/topic1", "/topic2"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (const rosbag::MessageInstance &m : view)
  {
    queue.push(m); // Enqueue message
  }
  bag.close();
}

int main(int argc, char **argv)
{
  ThreadSafeQueue<rosbag::MessageInstance> queue;

  // Read messages in main thread
  read_bag(queue);

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

  return 0;
}