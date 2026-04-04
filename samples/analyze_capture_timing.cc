// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "../src/usb/uvc/cyperstereo_api.h"

CYPERSTEREO_USE_NAMESPACE

namespace {

std::atomic<bool> g_should_stop{false};

void handleSignal(int) {
  g_should_stop.store(true);
}

bool isValidTimestamp(double timestamp) {
  return std::isfinite(timestamp) && timestamp > 1e-6;
}

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

struct Options {
  int width = 752;
  int height = 480;
  int fps = 50;
  double duration_sec = 20.0;
  cyperstereo::Format pixel_format = cyperstereo::Format::YUYV;
  bool verbose = false;
};

struct Summary {
  size_t count = 0;
  double mean = 0.0;
  double stddev = 0.0;
  double min = 0.0;
  double max = 0.0;
  double p50 = 0.0;
  double p90 = 0.0;
  double p99 = 0.0;
};

struct Counters {
  size_t image_count = 0;
  size_t imu_count = 0;
  size_t invalid_image_timestamp_count = 0;
  size_t invalid_imu_timestamp_count = 0;
  size_t non_monotonic_image_count = 0;
  size_t non_monotonic_imu_count = 0;
  size_t empty_imu_batch_count = 0;
};

struct TimingSeries {
  std::vector<double> image_dt_sec;
  std::vector<double> image_wall_dt_sec;
  std::vector<double> imu_dt_sec;
  std::vector<double> imu_per_image;
  std::vector<double> image_minus_earliest_imu_sec;
  std::vector<double> image_minus_latest_imu_sec;
  std::vector<double> image_minus_nearest_imu_abs_sec;
  std::vector<double> imu_batch_span_sec;
};

Summary summarize(const std::vector<double> &values) {
  Summary summary;
  summary.count = values.size();
  if (values.empty()) {
    return summary;
  }

  summary.min = *std::min_element(values.begin(), values.end());
  summary.max = *std::max_element(values.begin(), values.end());
  summary.mean = std::accumulate(values.begin(), values.end(), 0.0) /
      static_cast<double>(values.size());

  double variance = 0.0;
  for (const double value : values) {
    const double diff = value - summary.mean;
    variance += diff * diff;
  }
  variance /= static_cast<double>(values.size());
  summary.stddev = std::sqrt(variance);

  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());

  const auto percentile = [&sorted](double fraction) {
    const double scaled = fraction * static_cast<double>(sorted.size() - 1);
    const size_t index = static_cast<size_t>(std::llround(scaled));
    return sorted[index];
  };

  summary.p50 = percentile(0.50);
  summary.p90 = percentile(0.90);
  summary.p99 = percentile(0.99);
  return summary;
}

double rateFromTimestamps(size_t sample_count, double first_timestamp, double last_timestamp) {
  if (sample_count < 2 || !isValidTimestamp(first_timestamp) ||
      !isValidTimestamp(last_timestamp) || last_timestamp <= first_timestamp) {
    return 0.0;
  }

  return static_cast<double>(sample_count - 1) / (last_timestamp - first_timestamp);
}

void printSecondsSummary(const std::string &label, const std::vector<double> &values) {
  const Summary summary = summarize(values);
  std::cout << label << ": ";
  if (summary.count == 0) {
    std::cout << "no data" << std::endl;
    return;
  }

  std::cout << std::fixed << std::setprecision(3)
            << "count=" << summary.count
            << ", mean=" << summary.mean * 1000.0 << " ms"
            << ", std=" << summary.stddev * 1000.0 << " ms"
            << ", min=" << summary.min * 1000.0 << " ms"
            << ", p50=" << summary.p50 * 1000.0 << " ms"
            << ", p90=" << summary.p90 * 1000.0 << " ms"
            << ", p99=" << summary.p99 * 1000.0 << " ms"
            << ", max=" << summary.max * 1000.0 << " ms"
            << std::endl;
}

void printScalarSummary(const std::string &label, const std::vector<double> &values) {
  const Summary summary = summarize(values);
  std::cout << label << ": ";
  if (summary.count == 0) {
    std::cout << "no data" << std::endl;
    return;
  }

  std::cout << std::fixed << std::setprecision(3)
            << "count=" << summary.count
            << ", mean=" << summary.mean
            << ", std=" << summary.stddev
            << ", min=" << summary.min
            << ", p50=" << summary.p50
            << ", p90=" << summary.p90
            << ", p99=" << summary.p99
            << ", max=" << summary.max
            << std::endl;
}

Options parseOptions(int argc, char *argv[]) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--duration" && index + 1 < argc) {
      options.duration_sec = std::stod(argv[++index]);
      continue;
    }
    if (argument == "--fps" && index + 1 < argc) {
      options.fps = std::stoi(argv[++index]);
      continue;
    }
    if (argument == "--width" && index + 1 < argc) {
      options.width = std::stoi(argv[++index]);
      continue;
    }
    if (argument == "--height" && index + 1 < argc) {
      options.height = std::stoi(argv[++index]);
      continue;
    }
    if (argument == "--format" && index + 1 < argc) {
      const std::string format = toLower(argv[++index]);
      if (format == "yuyv") {
        options.pixel_format = cyperstereo::Format::YUYV;
      } else if (format == "gray" || format == "grey" || format == "mono8") {
        options.pixel_format = cyperstereo::Format::GREY;
      } else {
        throw std::invalid_argument("Unsupported --format value: " + format);
      }
      continue;
    }
    if (argument == "--verbose") {
      options.verbose = true;
      continue;
    }
    if (argument == "--help") {
      std::cout
          << "Usage: analyze_capture_timing [--duration SEC] [--fps N] [--width N] [--height N]"
          << " [--format yuyv|grey] [--verbose]" << std::endl;
      std::exit(0);
    }

    throw std::invalid_argument("Unknown argument: " + argument);
  }

  if (options.width <= 0 || options.height <= 0 || options.fps <= 0) {
    throw std::invalid_argument("width, height and fps must be positive.");
  }
  if (!std::isfinite(options.duration_sec) || options.duration_sec <= 0.0) {
    throw std::invalid_argument("duration must be positive.");
  }

  return options;
}

}  // namespace

int main(int argc, char *argv[]) {
  try {
    const Options options = parseOptions(argc, argv);
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::shared_ptr<cyperstereo::uvc::device> cyperstereo_device{nullptr};
    if (!cyperstereo::FindCyperstereoDevices(cyperstereo_device)) {
      return 1;
    }

    cyperstereo::FrameInfo frame_info{};
    cyperstereo::uvc::set_device_mode(
        *cyperstereo_device, options.width, options.height,
        static_cast<int>(options.pixel_format), options.fps,
        [&frame_info](const void *data, std::function<void()> continuation) {
          cyperstereo::SetStreamData(frame_info, data, continuation);
        });
    cyperstereo::uvc::start_streaming(*cyperstereo_device, 0);

    Counters counters;
    TimingSeries timing;
    std::map<int, size_t> imu_batch_histogram;

    double first_image_timestamp = 0.0;
    double last_image_timestamp = 0.0;
    double previous_image_timestamp = 0.0;

    double first_imu_timestamp = 0.0;
    double last_imu_timestamp = 0.0;
    double previous_imu_timestamp = 0.0;

    bool has_previous_wall_image = false;
    auto previous_wall_image_time = std::chrono::steady_clock::now();
    const auto wall_begin = std::chrono::steady_clock::now();

    while (!g_should_stop.load()) {
      cyperstereo::WaitForStream(frame_info);
      const auto wall_now = std::chrono::steady_clock::now();

      double image_timestamp = 0.0;
      cyperstereo::IMUStreamData imu_data{};
      {
        std::lock_guard<std::mutex> lock(frame_info.mtx);
        image_timestamp = frame_info.framestream.image_timestamp;
        imu_data = frame_info.framestream.imu;
      }

      if (isValidTimestamp(image_timestamp)) {
        ++counters.image_count;
        if (!isValidTimestamp(first_image_timestamp)) {
          first_image_timestamp = image_timestamp;
        }
        last_image_timestamp = image_timestamp;

        if (isValidTimestamp(previous_image_timestamp)) {
          const double image_dt = image_timestamp - previous_image_timestamp;
          if (image_dt > 0.0) {
            timing.image_dt_sec.push_back(image_dt);
          } else {
            ++counters.non_monotonic_image_count;
          }
        }
        previous_image_timestamp = image_timestamp;

        if (has_previous_wall_image) {
          const double wall_dt = std::chrono::duration<double>(
              wall_now - previous_wall_image_time).count();
          if (wall_dt > 0.0) {
            timing.image_wall_dt_sec.push_back(wall_dt);
          }
        }
        previous_wall_image_time = wall_now;
        has_previous_wall_image = true;
      } else {
        ++counters.invalid_image_timestamp_count;
      }

      std::vector<double> batch_imu_timestamps;
      const int raw_imu_count = std::max(0, imu_data.imu_count + 1);
      batch_imu_timestamps.reserve(static_cast<size_t>(raw_imu_count));
      for (int index = 0; index < raw_imu_count; ++index) {
        const double imu_timestamp = imu_data.imu_timestamp[index];
        if (!isValidTimestamp(imu_timestamp)) {
          ++counters.invalid_imu_timestamp_count;
          continue;
        }

        batch_imu_timestamps.push_back(imu_timestamp);
        ++counters.imu_count;

        if (!isValidTimestamp(first_imu_timestamp)) {
          first_imu_timestamp = imu_timestamp;
        }
        last_imu_timestamp = imu_timestamp;

        if (isValidTimestamp(previous_imu_timestamp)) {
          const double imu_dt = imu_timestamp - previous_imu_timestamp;
          if (imu_dt > 0.0) {
            timing.imu_dt_sec.push_back(imu_dt);
          } else {
            ++counters.non_monotonic_imu_count;
          }
        }
        previous_imu_timestamp = imu_timestamp;
      }

      if (batch_imu_timestamps.empty()) {
        ++counters.empty_imu_batch_count;
      } else {
        imu_batch_histogram[static_cast<int>(batch_imu_timestamps.size())] += 1;
        timing.imu_per_image.push_back(static_cast<double>(batch_imu_timestamps.size()));
      }

      if (isValidTimestamp(image_timestamp) && !batch_imu_timestamps.empty()) {
        const double earliest_imu = batch_imu_timestamps.front();
        const double latest_imu = batch_imu_timestamps.back();
        timing.image_minus_earliest_imu_sec.push_back(image_timestamp - earliest_imu);
        timing.image_minus_latest_imu_sec.push_back(image_timestamp - latest_imu);
        timing.imu_batch_span_sec.push_back(latest_imu - earliest_imu);

        double nearest_abs_offset = std::numeric_limits<double>::infinity();
        for (const double imu_timestamp : batch_imu_timestamps) {
          nearest_abs_offset = std::min(nearest_abs_offset,
              std::abs(image_timestamp - imu_timestamp));
        }
        if (std::isfinite(nearest_abs_offset)) {
          timing.image_minus_nearest_imu_abs_sec.push_back(nearest_abs_offset);
        }
      }

      if (options.verbose && isValidTimestamp(image_timestamp)) {
        std::cout << std::fixed << std::setprecision(6)
                  << "image_ts=" << image_timestamp
                  << ", imu_in_batch=" << batch_imu_timestamps.size();
        if (!batch_imu_timestamps.empty()) {
          std::cout << ", earliest_imu_dt_ms="
                    << (image_timestamp - batch_imu_timestamps.front()) * 1000.0
                    << ", latest_imu_dt_ms="
                    << (image_timestamp - batch_imu_timestamps.back()) * 1000.0;
        }
        std::cout << std::endl;
      }

      const double elapsed_sec = std::chrono::duration<double>(
          wall_now - wall_begin).count();
      if (elapsed_sec >= options.duration_sec) {
        break;
      }
    }

    cyperstereo::uvc::stop_streaming(*cyperstereo_device);

    const double image_rate_device = rateFromTimestamps(
        counters.image_count, first_image_timestamp, last_image_timestamp);
    const double imu_rate_device = rateFromTimestamps(
        counters.imu_count, first_imu_timestamp, last_imu_timestamp);
    const double image_rate_wall = summarize(timing.image_wall_dt_sec).count > 0
        ? 1.0 / summarize(timing.image_wall_dt_sec).mean
        : 0.0;
    const double imu_per_image_by_count = counters.image_count > 0
        ? static_cast<double>(counters.imu_count) /
            static_cast<double>(counters.image_count)
        : 0.0;
    const double imu_per_image_by_rate = image_rate_device > 0.0
        ? imu_rate_device / image_rate_device
        : 0.0;

    std::cout << std::endl;
    std::cout << "=== Capture Timing Analysis ===" << std::endl;
    std::cout << "requested_mode: width=" << options.width
              << ", height=" << options.height
              << ", fps=" << options.fps
              << ", pixel_format="
              << (options.pixel_format == cyperstereo::Format::GREY ? "GREY" : "YUYV")
              << std::endl;
    std::cout << "valid_image_samples=" << counters.image_count
              << ", valid_imu_samples=" << counters.imu_count
              << ", invalid_image_timestamps=" << counters.invalid_image_timestamp_count
              << ", invalid_imu_timestamps=" << counters.invalid_imu_timestamp_count
              << std::endl;
    std::cout << "non_monotonic_image_timestamps=" << counters.non_monotonic_image_count
              << ", non_monotonic_imu_timestamps=" << counters.non_monotonic_imu_count
              << ", empty_imu_batches=" << counters.empty_imu_batch_count
              << std::endl;

    std::cout << std::fixed << std::setprecision(3)
              << "image_rate_device_ts=" << image_rate_device << " Hz"
              << ", image_rate_wall=" << image_rate_wall << " Hz"
              << ", imu_rate_device_ts=" << imu_rate_device << " Hz"
              << std::endl;
    std::cout << "imu_per_image_by_count=" << imu_per_image_by_count
              << ", imu_per_image_by_rate=" << imu_per_image_by_rate;
    if (imu_per_image_by_rate > 0.0) {
      std::cout << ", cam:imu ~= 1:" << imu_per_image_by_rate;
    }
    std::cout << std::endl;

    std::cout << "imu_batch_histogram:";
    if (imu_batch_histogram.empty()) {
      std::cout << " no data" << std::endl;
    } else {
      std::cout << std::endl;
      for (const auto &entry : imu_batch_histogram) {
        std::cout << "  " << entry.first << " imu/frame -> " << entry.second << " batches"
                  << std::endl;
      }
    }

    printSecondsSummary("image_dt_device", timing.image_dt_sec);
    printSecondsSummary("image_dt_wall", timing.image_wall_dt_sec);
    printSecondsSummary("imu_dt_device", timing.imu_dt_sec);
    printScalarSummary("imu_samples_per_image", timing.imu_per_image);
    printSecondsSummary("image_minus_earliest_imu", timing.image_minus_earliest_imu_sec);
    printSecondsSummary("image_minus_latest_imu", timing.image_minus_latest_imu_sec);
    printSecondsSummary("image_minus_nearest_imu_abs", timing.image_minus_nearest_imu_abs_sec);
    printSecondsSummary("imu_batch_span", timing.imu_batch_span_sec);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "analyze_capture_timing failed: " << error.what() << std::endl;
    return 1;
  }
}