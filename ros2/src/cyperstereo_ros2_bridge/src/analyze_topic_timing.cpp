#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace {

double ToSeconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

struct StreamStats {
  uint64_t total_count = 0;
  uint64_t invalid_count = 0;
  uint64_t non_monotonic_count = 0;
  double first_stamp = 0.0;
  double last_stamp = 0.0;
  double previous_stamp = 0.0;
};

struct PairingStats {
  bool found = false;
  size_t left_index = 0;
  size_t right_index = 0;
  double best_delta_sec = std::numeric_limits<double>::max();
  double left_stamp = 0.0;
  double right_stamp = 0.0;
};

bool IsValidStamp(double stamp) {
  return std::isfinite(stamp) && stamp > 1e-6;
}

double ComputeRateHz(const StreamStats &stats) {
  if (stats.total_count < 2 || !IsValidStamp(stats.first_stamp) ||
      !IsValidStamp(stats.last_stamp) || stats.last_stamp <= stats.first_stamp) {
    return 0.0;
  }

  return static_cast<double>(stats.total_count - 1) /
         (stats.last_stamp - stats.first_stamp);
}

PairingStats FindBestStereoPair(const std::deque<double> &left_queue,
                                const std::deque<double> &right_queue,
                                size_t search_window) {
  PairingStats stats;
  if (left_queue.empty() || right_queue.empty()) {
    return stats;
  }

  const size_t left_count = std::min(left_queue.size(), search_window);
  const size_t right_count = std::min(right_queue.size(), search_window);
  for (size_t left_candidate = 0; left_candidate < left_count; ++left_candidate) {
    for (size_t right_candidate = 0; right_candidate < right_count;
         ++right_candidate) {
      const double left_stamp = left_queue[left_candidate];
      const double right_stamp = right_queue[right_candidate];
      const double delta_sec = std::fabs(left_stamp - right_stamp);
      if (delta_sec < stats.best_delta_sec) {
        stats.best_delta_sec = delta_sec;
        stats.left_index = left_candidate;
        stats.right_index = right_candidate;
        stats.left_stamp = left_stamp;
        stats.right_stamp = right_stamp;
        stats.found = true;
      }
    }
  }

  return stats;
}

class CyperstereoTopicTimingAnalyzer : public rclcpp::Node {
public:
  CyperstereoTopicTimingAnalyzer()
      : Node("cyperstereo_topic_timing_analyzer") {
    left_topic_ = declare_parameter<std::string>("left_topic", "/cam0/image_raw");
    right_topic_ =
        declare_parameter<std::string>("right_topic", "/cam1/image_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu0");
    image_queue_size_ = declare_parameter<int>("image_queue_size", 120);
    imu_queue_size_ = declare_parameter<int>("imu_queue_size", 1000);
    search_window_ = declare_parameter<int>("search_window", 30);
    sync_tolerance_ms_ = declare_parameter<double>("sync_tolerance_ms", 100.0);
    stats_period_sec_ = declare_parameter<double>("stats_period_sec", 2.0);
    const int image_depth = declare_parameter<int>("qos.image.depth", 10);
    const int imu_depth = declare_parameter<int>("qos.imu.depth", 1000);
    const std::string image_reliability =
        declare_parameter<std::string>("qos.image.reliability", "reliable");
    const std::string imu_reliability =
        declare_parameter<std::string>("qos.imu.reliability", "reliable");

    left_sub_ = create_subscription<sensor_msgs::msg::Image>(
        left_topic_, MakeQos(image_depth, image_reliability),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { OnLeftImage(msg); });
    right_sub_ = create_subscription<sensor_msgs::msg::Image>(
        right_topic_, MakeQos(image_depth, image_reliability),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { OnRightImage(msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, MakeQos(imu_depth, imu_reliability),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImu(msg); });

    stats_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(stats_period_sec_)),
        [this]() { ReportStats(); });

    RCLCPP_INFO(
        get_logger(),
        "Topic timing analyzer started. left=%s right=%s imu=%s image_queue=%d imu_queue=%d search_window=%d sync_tolerance_ms=%.3f",
        left_topic_.c_str(), right_topic_.c_str(), imu_topic_.c_str(),
        image_queue_size_, imu_queue_size_, search_window_, sync_tolerance_ms_);
  }

private:
  rclcpp::QoS MakeQos(int depth, const std::string &reliability) const {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(std::max(1, depth)));
    if (reliability == "best_effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    qos.durability_volatile();
    return qos;
  }

  void UpdateStreamStats(double stamp, StreamStats &stats) {
    if (!IsValidStamp(stamp)) {
      ++stats.invalid_count;
      return;
    }

    ++stats.total_count;
    if (!IsValidStamp(stats.first_stamp)) {
      stats.first_stamp = stamp;
    }
    if (IsValidStamp(stats.previous_stamp) && stamp <= stats.previous_stamp) {
      ++stats.non_monotonic_count;
    }
    stats.previous_stamp = stamp;
    stats.last_stamp = stamp;
  }

  void PushStamp(double stamp, std::deque<double> &queue, size_t limit) {
    if (!IsValidStamp(stamp)) {
      return;
    }
    queue.push_back(stamp);
    while (queue.size() > limit) {
      queue.pop_front();
    }
  }

  void OnLeftImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    const double stamp = ToSeconds(msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    UpdateStreamStats(stamp, left_stats_);
    PushStamp(stamp, left_queue_, static_cast<size_t>(std::max(1, image_queue_size_)));
  }

  void OnRightImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    const double stamp = ToSeconds(msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    UpdateStreamStats(stamp, right_stats_);
    PushStamp(stamp, right_queue_, static_cast<size_t>(std::max(1, image_queue_size_)));
  }

  void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
    const double stamp = ToSeconds(msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    UpdateStreamStats(stamp, imu_stats_);
    PushStamp(stamp, imu_queue_, static_cast<size_t>(std::max(1, imu_queue_size_)));
  }

  void ReportStats() {
    StreamStats left_stats;
    StreamStats right_stats;
    StreamStats imu_stats;
    std::deque<double> left_queue;
    std::deque<double> right_queue;
    std::deque<double> imu_queue;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      left_stats = left_stats_;
      right_stats = right_stats_;
      imu_stats = imu_stats_;
      left_queue = left_queue_;
      right_queue = right_queue_;
      imu_queue = imu_queue_;
    }

    const PairingStats pair_stats =
        FindBestStereoPair(left_queue, right_queue,
                           static_cast<size_t>(std::max(1, search_window_)));

    double front_delta_ms = -1.0;
    if (!left_queue.empty() && !right_queue.empty()) {
      front_delta_ms = std::fabs(left_queue.front() - right_queue.front()) * 1e3;
    }

    double latest_lr_delta_ms = -1.0;
    if (IsValidStamp(left_stats.last_stamp) && IsValidStamp(right_stats.last_stamp)) {
      latest_lr_delta_ms =
          std::fabs(left_stats.last_stamp - right_stats.last_stamp) * 1e3;
    }

    double latest_image_imu_delta_ms = -1.0;
    const double latest_image_stamp =
        std::max(left_stats.last_stamp, right_stats.last_stamp);
    if (IsValidStamp(latest_image_stamp) && IsValidStamp(imu_stats.last_stamp)) {
      latest_image_imu_delta_ms =
          (latest_image_stamp - imu_stats.last_stamp) * 1e3;
    }

    RCLCPP_INFO(
        get_logger(),
        "rates: left=%.2fHz right=%.2fHz imu=%.2fHz counts=[%llu,%llu,%llu] queues=[%zu,%zu,%zu] invalid=[%llu,%llu,%llu] nonmono=[%llu,%llu,%llu]",
        ComputeRateHz(left_stats), ComputeRateHz(right_stats),
        ComputeRateHz(imu_stats),
        static_cast<unsigned long long>(left_stats.total_count),
        static_cast<unsigned long long>(right_stats.total_count),
        static_cast<unsigned long long>(imu_stats.total_count), left_queue.size(),
        right_queue.size(), imu_queue.size(),
        static_cast<unsigned long long>(left_stats.invalid_count),
        static_cast<unsigned long long>(right_stats.invalid_count),
        static_cast<unsigned long long>(imu_stats.invalid_count),
        static_cast<unsigned long long>(left_stats.non_monotonic_count),
        static_cast<unsigned long long>(right_stats.non_monotonic_count),
        static_cast<unsigned long long>(imu_stats.non_monotonic_count));

    if (!pair_stats.found) {
      RCLCPP_WARN(get_logger(),
                  "stereo pairing: insufficient data left_queue=%zu right_queue=%zu",
                  left_queue.size(), right_queue.size());
      return;
    }

    const double best_delta_ms = pair_stats.best_delta_sec * 1e3;
    const bool exceeds_tolerance = best_delta_ms > sync_tolerance_ms_;
    if (exceeds_tolerance) {
      RCLCPP_WARN(
          get_logger(),
          "stereo pairing: best_delta_ms=%.3f exceeds tolerance=%.3f front_delta_ms=%.3f latest_lr_delta_ms=%.3f best_pair=(%zu,%zu) left_stamp=%.6f right_stamp=%.6f latest_image_minus_imu_ms=%.3f",
          best_delta_ms, sync_tolerance_ms_, front_delta_ms,
          latest_lr_delta_ms, pair_stats.left_index, pair_stats.right_index,
          pair_stats.left_stamp, pair_stats.right_stamp,
          latest_image_imu_delta_ms);
    } else {
      RCLCPP_INFO(
          get_logger(),
          "stereo pairing: best_delta_ms=%.3f front_delta_ms=%.3f latest_lr_delta_ms=%.3f best_pair=(%zu,%zu) left_stamp=%.6f right_stamp=%.6f latest_image_minus_imu_ms=%.3f",
          best_delta_ms, front_delta_ms, latest_lr_delta_ms,
          pair_stats.left_index, pair_stats.right_index, pair_stats.left_stamp,
          pair_stats.right_stamp, latest_image_imu_delta_ms);
    }
  }

  std::mutex mutex_;
  std::string left_topic_;
  std::string right_topic_;
  std::string imu_topic_;
  int image_queue_size_ = 120;
  int imu_queue_size_ = 1000;
  int search_window_ = 30;
  double sync_tolerance_ms_ = 100.0;
  double stats_period_sec_ = 2.0;

  StreamStats left_stats_;
  StreamStats right_stats_;
  StreamStats imu_stats_;
  std::deque<double> left_queue_;
  std::deque<double> right_queue_;
  std::deque<double> imu_queue_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CyperstereoTopicTimingAnalyzer>());
  rclcpp::shutdown();
  return 0;
}