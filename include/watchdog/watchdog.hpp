#ifndef WATCHDOG__WATCHDOG_HPP_
#define WATCHDOG__WATCHDOG_HPP_

#include <sys/types.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <functional>
#include <csignal>

namespace watchdog
{

class Watchdog
{
public:
  Watchdog();
  explicit Watchdog(std::function<void()> _callback);
  ~Watchdog();
  void start(unsigned int _interval);
  void stop();
  void refresh();

private:
  unsigned int interval;
  std::atomic<bool> isRunning;
  std::thread thread;
  std::function<void()> callback;
  std::mutex mutex;
  std::chrono::steady_clock::time_point lastRefreshTime;
  std::condition_variable stopCondition;
  void loop();
};

}  // namespace watchdog

#endif  // WATCHDOG__WATCHDOG_HPP_
