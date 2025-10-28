/*
  Example code for C++ multithreading features:
  - std::this_thread::sleep_for
  - std::future and std::async
  - std::promise
  - std::atomic
  - std::condition_variable
*/


#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <algorithm>
#include <chrono>
#include <iostream>

int task()
{
  std::this_thread::sleep_for(std::chrono::seconds(2));
  return 42;
}

void task_promise(std::promise<void> done_promise)
{
  std::this_thread::sleep_for(std::chrono::seconds(2));
  done_promise.set_value(); // signal completion
}

void worker_atomic(int id, std::atomic<bool> &done)
{
  // ... do work ...
  std::this_thread::sleep_for(std::chrono::milliseconds(200 + id * 50));
  done.store(true, std::memory_order_release); // signal completion
}

void worker_condition(std::atomic<int> &remaining, std::mutex &m, std::condition_variable &cv)
{
  // ... do work ...
  // signal
  if (remaining.fetch_sub(1, std::memory_order_acq_rel) == 1)
  {
    std::lock_guard<std::mutex> lk(m);
    cv.notify_all(); // last one wakes the waiter
  }
}

int main(int argc, char **argv)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5 sec
  std::this_thread::sleep_for(std::chrono::seconds(1));        // 1 sec
  std::this_thread::sleep_for(std::chrono::minutes(2));        // 2 minutes

  // use std::future to join and get result
  {
    std::future<int> fut = std::async(std::launch::async, task); // std::async: launches a function asynchronously

    // wait with timeout
    if (fut.wait_until(std::chrono::steady_clock::now() + std::chrono::seconds(3)) == std::future_status::ready)
    {
      std::cout << "Task finished within 3 seconds\n";
    }
    else
    {
      std::cout << "Task still running after 3 seconds\n";
    }

    // Wait until ready
    if (fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      std::cout << "Task finished\n";
    }
    else
    {
      std::cout << "Task still running\n";
    }


    // if without wait above, this will block until ready
    std::cout << "Result = " << fut.get() << "\n";
  }

  // use std::promise and std::future for signaling
  {
    std::promise<void> p;
    std::future<void> f = p.get_future();

    std::thread t(task_promise, std::move(p));

    if (f.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      std::cout << "Finished\n";
    }
    else
    {
      std::cout << "Not yet\n";
    }

    f.wait(); // ensure done
    t.join();
  }

  // judge completion with std::atomic
  {
    const std::size_t N = 5;
    std::vector<std::thread> threads;
    std::vector<std::atomic<bool>> done(N); // default false
    for (auto &f : done) f.store(false, std::memory_order_relaxed);

    threads.reserve(N);
    for (std::size_t i = 0; i < N; ++i)
    {
      threads.emplace_back(worker_atomic, static_cast<int>(i), std::ref(done[i]));
    }

    // Poll (avoid tight spin in real code)
    while (!std::all_of(done.begin(), done.end(),
                        [](const std::atomic<bool> &f) { return f.load(std::memory_order_acquire); }))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "All tasks finished\n";

    for (auto &t : threads)
    {
      if (t.joinable()) t.join();
    }
  }

  // judge completion with std::atomic and std::condition_variable
  {
    const int N = 5;
    std::atomic<int> remaining{N};
    std::mutex m;
    std::condition_variable cv;

    std::vector<std::thread> threads;
    threads.reserve(N);
    for (int i = 0; i < N; ++i)
    {
      threads.emplace_back(worker_condition, std::ref(remaining), std::ref(m), std::ref(cv));
    }

    // Wait without busy-wait
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, [&] { return remaining.load(std::memory_order_acquire) == 0; });

    for (auto &t : threads)
    {
      if (t.joinable()) t.join();
    }
  }

  // Using multiple threads to perform work and signal completion
  {
    const std::size_t N = 5;
    std::vector<std::thread> threads;
    std::vector<std::future<void>> futures;
    threads.reserve(N);
    futures.reserve(N);

    for (std::size_t i = 0; i < N; ++i)
    {
      std::promise<void> p;
      futures.push_back(p.get_future());
      threads.emplace_back(worker_promise, std::move(p));
    }

    // Wait for all
    for (auto &f : futures) f.wait();

    for (auto &t : threads)
    {
      if (t.joinable()) t.join();
    }
  }

  // before overwriting a slot, the existing thread mustn't be joinable (already joined or detached). Otherwise the program will call std::terminate():

  return 0;
}