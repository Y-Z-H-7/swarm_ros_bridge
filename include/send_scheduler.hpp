#ifndef SWARM_ROS_BRIDGE_SEND_SCHEDULER_HPP
#define SWARM_ROS_BRIDGE_SEND_SCHEDULER_HPP

#include "bridge_node.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <stdexcept>

// The scheduler owns its PUB sockets in one thread. ROS callbacks only enqueue.
class SendScheduler
{
public:
  SendScheduler(zmqpp::context_t &context, const std::vector<TopicInfo> &topics,
                double budget_bytes_per_second, size_t burst_bytes)
      : context_(context), topics_(topics), budget_(budget_bytes_per_second),
        burst_(burst_bytes), tokens_(burst_bytes), queues_(topics.size()),
        queued_bytes_(topics.size(), 0), classes_(topics.size()), latest_(topics.size(), false)
  {
    if (budget_ <= 0 || burst_ == 0) throw std::invalid_argument("scheduler budget/burst must be positive");
    for (size_t i = 0; i < topics_.size(); ++i) {
      classes_[i] = classify(topics_[i].name);
      latest_[i] = is_latest(topics_[i].name);
      ROS_INFO("[bridge scheduler] %s -> P%d (%s)", topics_[i].name.c_str(),
               classes_[i], latest_[i] ? "latest" : "fifo");
    }
  }

  void start()
  {
    worker_ = std::thread(&SendScheduler::run, this);
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return ready_; });
    if (start_error_) {
      lock.unlock();
      worker_.join();
      std::rethrow_exception(start_error_);
    }
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    cv_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

  ~SendScheduler() { stop(); }

  void enqueue(size_t topic, const uint8_t *data, size_t length)
  {
    if (topic >= queues_.size()) return;
    Frame frame;
    frame.data.assign(data, data + length);
    frame.created = Clock::now();
    const int cls = classes_[topic];
    const size_t cap = cap_bytes(cls);
    std::lock_guard<std::mutex> lock(mutex_);
    auto &queue = queues_[topic];
    if (latest_[topic]) {
      while (!queue.empty()) {
        queued_bytes_[topic] -= queue.front().data.size();
        queue.pop_front();
        ++overwritten_;
      }
    }
    if (length > burst_ || length > cap || queued_bytes_[topic] + length > cap ||
        (!latest_[topic] && queue.size() >= cap_messages(cls))) {
      ++rejected_;
      ROS_WARN_THROTTLE(5.0, "[bridge scheduler] queue full: %s, rejected=%llu",
                        topics_[topic].name.c_str(), static_cast<unsigned long long>(rejected_));
      return;
    }
    queued_bytes_[topic] += length;
    queue.push_back(std::move(frame));
    cv_.notify_one();
  }

private:
  using Clock = std::chrono::steady_clock;
  struct Frame {
    std::vector<uint8_t> data;
    Clock::time_point created;
  };
  static size_t cap_bytes(int cls)
  {
    const size_t caps[] = {65536, 524288, 131072, 4194304, 4194304};
    return caps[cls];
  }
  static size_t cap_messages(int cls)
  {
    const size_t caps[] = {32, 128, 16, 1024, 32};
    return caps[cls];
  }
  static int weight(int cls)
  {
    const int weights[] = {0, 8, 4, 2, 1};
    return weights[cls];
  }

  static int classify(const std::string &name)
  {
    if (name.find("/move_base_simple/goal") != std::string::npos ||
        name.find("/mission/land_request") != std::string::npos) return 0;
    if (name.find("swarm_traj_send") != std::string::npos ||
        name.find("drone_state_send") != std::string::npos ||
        name.find("pair_opt") != std::string::npos) return 1;
    if (name.find("/mavros/battery") != std::string::npos ||
        name.find("/mavros/local_position/pose") != std::string::npos) return 2;
    if (name.find("chunk_stamps_send") != std::string::npos ||
        name.find("chunk_data_send") != std::string::npos ||
        name.find("hgrid_send") != std::string::npos ||
        name.find("grid_tour_send") != std::string::npos) return 3;
    return 4;
  }

  static bool is_latest(const std::string &name)
  {
    return name.find("drone_state_send") != std::string::npos ||
           name.find("swarm_traj_send") != std::string::npos ||
           name.find("/mavros/") != std::string::npos ||
           name.find("chunk_stamps_send") != std::string::npos ||
           name.find("occupancy_all") != std::string::npos;
  }

  bool any_pending() const
  {
    for (const auto &q : queues_) if (!q.empty()) return true;
    return false;
  }

  // Strict P0 service, then byte-weighted deficit round robin for P1-P4.
  int choose_topic()
  {
    for (size_t i = 0; i < queues_.size(); ++i)
      if (classes_[i] == 0 && !queues_[i].empty() &&
          queues_[i].front().data.size() <= tokens_) return static_cast<int>(i);
    for (int n = 0; n < 4; ++n) {
      const int cls = class_cursor_;
      bool pending = false;
      for (size_t i = 0; i < queues_.size(); ++i)
        if (classes_[i] == cls && !queues_[i].empty()) { pending = true; break; }
      if (!pending) {
        deficit_[cls] = 0;
        credited_[cls] = false;
        class_cursor_ = 1 + cls % 4;
        continue;
      }
      if (!credited_[cls]) {
        deficit_[cls] = std::min<double>(burst_, deficit_[cls] + 16384.0 * weight(cls));
        credited_[cls] = true;
      }
      bool credit_sufficient = false;
      for (size_t j = 0; j < queues_.size(); ++j) {
        size_t i = (topic_cursor_[cls] + j) % queues_.size();
        if (classes_[i] != cls || queues_[i].empty()) continue;
        const size_t size = queues_[i].front().data.size();
        if (size <= deficit_[cls]) credit_sufficient = true;
        if (size <= deficit_[cls] && size <= tokens_) {
          deficit_[cls] -= size;
          topic_cursor_[cls] = (i + 1) % queues_.size();
          return static_cast<int>(i);
        }
      }
      // A new round adds credit only when this class has exhausted its share.
      if (!credit_sufficient) credited_[cls] = false;
      class_cursor_ = 1 + cls % 4;
    }
    return -1;
  }

  void run()
  {
    std::vector<std::unique_ptr<zmqpp::socket>> sockets;
    try {
      for (const auto &topic : topics_) {
        std::unique_ptr<zmqpp::socket> socket(new zmqpp::socket(context_, zmqpp::socket_type::pub));
        socket->bind("tcp://" + topic.ip + ":" + std::to_string(topic.port));
        sockets.push_back(std::move(socket));
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      start_error_ = std::current_exception();
      ready_ = true;
      cv_.notify_all();
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready_ = true;
    }
    cv_.notify_all();
    auto last = Clock::now();
    auto last_report = last;
    while (true) {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait_for(lock, std::chrono::milliseconds(5), [this] { return stopping_ || any_pending(); });
      if (stopping_) break;
      auto now = Clock::now();
      tokens_ = std::min<double>(burst_, tokens_ + budget_ * std::chrono::duration<double>(now - last).count());
      last = now;
      int topic = choose_topic();
      if (topic < 0) {
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      Frame frame = std::move(queues_[topic].front());
      queues_[topic].pop_front();
      queued_bytes_[topic] -= frame.data.size();
      tokens_ -= frame.data.size();
      lock.unlock();
      zmqpp::message message;
      message << frame.data.size();
      message.add_raw(frame.data.data(), frame.data.size());
      sockets[topic]->send(message, false);
      ++sent_;
      if (now - last_report >= std::chrono::seconds(5)) {
        ROS_INFO("[bridge scheduler] sent=%llu overwritten=%llu rejected=%llu",
                 static_cast<unsigned long long>(sent_),
                 static_cast<unsigned long long>(overwritten_),
                 static_cast<unsigned long long>(rejected_));
        last_report = now;
      }
    }
    for (auto &socket : sockets) socket->close();
  }

  zmqpp::context_t &context_;
  const std::vector<TopicInfo> &topics_;
  const double budget_;
  const size_t burst_;
  double tokens_;
  std::vector<std::deque<Frame>> queues_;
  std::vector<size_t> queued_bytes_;
  std::vector<int> classes_;
  std::vector<bool> latest_;
  std::array<double, 5> deficit_{{0, 0, 0, 0, 0}};
  std::array<size_t, 5> topic_cursor_{{0, 0, 0, 0, 0}};
  std::array<bool, 5> credited_{{false, false, false, false, false}};
  int class_cursor_ = 1;
  std::atomic<unsigned long long> overwritten_{0}, rejected_{0}, sent_{0};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool ready_ = false, stopping_ = false;
  std::exception_ptr start_error_;
};

#endif
