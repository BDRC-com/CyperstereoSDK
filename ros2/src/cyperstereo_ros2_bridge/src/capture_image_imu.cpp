#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "../../../../src/usb/uvc/cyperstereo_api.h"

CYPERSTEREO_USE_NAMESPACE

using namespace std::chrono_literals;

namespace {

struct TopicQosConfig {
    std::string history;
    int depth;
    std::string reliability;
    std::string durability;
};

struct DeviceConfig {
    int width;
    int height;
    int fps;
    int stream_index;
    std::string pixel_format;
};

struct CaptureConfig {
    int reconnect_delay_ms;
    int queue_size;
    std::string queue_drop_policy;
};

struct ImageStreamConfig {
    std::string topic;
    std::string frame_id;
    std::string encoding;
};

struct ImageConfig {
    ImageStreamConfig left;
    ImageStreamConfig right;
    double publish_hz;
};

struct ImuConfig {
    std::string topic;
    std::string frame_id;
    double gravity_magnitude;
    double publish_hz;
};

struct DebugConfig {
    bool log_image_timestamps;
    bool log_imu_timestamps;
};

struct ImuSample {
    double timestamp = 0.0;
    double gyro_x = 0.0;
    double gyro_y = 0.0;
    double gyro_z = 0.0;
    double acc_x = 0.0;
    double acc_y = 0.0;
    double acc_z = 0.0;
};

struct CaptureBatch {
    double image_timestamp = 0.0;
    cv::Mat left_image;
    cv::Mat right_image;
    std::vector<ImuSample> imu_samples;
};

std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

rclcpp::Time toRosTime(double seconds) {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

bool isValidDeviceTimestamp(double timestamp) {
    return std::isfinite(timestamp) && timestamp > 1e-6;
}

cyperstereo::Format resolvePixelFormat(const std::string &pixel_format) {
    const std::string normalized = toLowerCopy(pixel_format);
    if (normalized == "yuyv") {
        return cyperstereo::Format::YUYV;
    }
    return cyperstereo::Format::YUYV;
}

    bool shouldDropOldest(const std::string &drop_policy) {
        return toLowerCopy(drop_policy) != "drop_newest";
    }

    bool shouldPublishTimestamp(double timestamp, double target_hz,
                               double &last_published_timestamp) {
        if (!isValidDeviceTimestamp(timestamp)) {
            return false;
        }

        if (!(target_hz > 0.0) || !std::isfinite(target_hz)) {
            last_published_timestamp = timestamp;
            return true;
        }

        if (!std::isfinite(last_published_timestamp)) {
            last_published_timestamp = timestamp;
            return true;
        }

        const double min_period = 1.0 / target_hz;
        if (timestamp + 1e-6 < last_published_timestamp + min_period) {
            return false;
        }

        last_published_timestamp = timestamp;
        return true;
    }

}  // namespace

    class CyperstereoRos2BridgeNode : public rclcpp::Node {
 public:
        CyperstereoRos2BridgeNode() : Node("cyperstereo_ros2_bridge"), is_running_(true) {
        device_config_ = loadDeviceConfig();
        capture_config_ = loadCaptureConfig();
        image_config_ = loadImageConfig();
        imu_config_ = loadImuConfig();
        debug_config_ = loadDebugConfig();

        cam0_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            image_config_.left.topic, loadQosConfig("qos.left_image", 10));
        cam1_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            image_config_.right.topic, loadQosConfig("qos.right_image", 10));
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            imu_config_.topic, loadQosConfig("qos.imu", 1000));

        producer_thread_ = std::thread(&CyperstereoRos2BridgeNode::captureLoop, this);
        consumer_thread_ = std::thread(&CyperstereoRos2BridgeNode::publishLoop, this);

        RCLCPP_INFO(
                get_logger(),
            "Bridge initialized. left=%s right=%s imu=%s, capture=%dx%d@%dfps, publish image=%.2fHz imu=%.2fHz, queue=%d/%s",
            image_config_.left.topic.c_str(), image_config_.right.topic.c_str(),
            imu_config_.topic.c_str(), device_config_.width,
            device_config_.height, device_config_.fps,
            image_config_.publish_hz, imu_config_.publish_hz,
            capture_config_.queue_size, capture_config_.queue_drop_policy.c_str());
    }

        ~CyperstereoRos2BridgeNode() override {
        is_running_ = false;
        queue_cv_.notify_all();

        if (producer_thread_.joinable()) {
            producer_thread_.join();
        }
        if (consumer_thread_.joinable()) {
            consumer_thread_.join();
        }

        RCLCPP_INFO(get_logger(), "Data publisher node threads exited.");
    }

 private:
        DeviceConfig loadDeviceConfig() {
        DeviceConfig config{};
        config.width = declare_parameter<int>("device.width", 752);
        config.height = declare_parameter<int>("device.height", 480);
        config.fps = declare_parameter<int>("device.fps", 60);
        config.stream_index = declare_parameter<int>("device.stream_index", 0);
        config.pixel_format =
                declare_parameter<std::string>("device.pixel_format", "YUYV");
        return config;
    }

        CaptureConfig loadCaptureConfig() {
        CaptureConfig config{};
        config.reconnect_delay_ms =
            declare_parameter<int>("capture.reconnect_delay_ms", 1000);
        config.queue_size = declare_parameter<int>("capture.queue.size", 8);
        config.queue_drop_policy = declare_parameter<std::string>(
            "capture.queue.drop_policy", "drop_oldest");
        return config;
    }

        ImageConfig loadImageConfig() {
        ImageConfig config{};
        config.left.topic = declare_parameter<std::string>(
            "image.left.topic", "/cam0/image_raw");
        config.left.frame_id = declare_parameter<std::string>(
            "image.left.frame_id", "cam0");
        config.left.encoding = declare_parameter<std::string>(
            "image.left.encoding", "mono8");
        config.publish_hz = declare_parameter<double>("image.publish_hz", 20.0);

        config.right.topic = declare_parameter<std::string>(
            "image.right.topic", "/cam1/image_raw");
        config.right.frame_id = declare_parameter<std::string>(
            "image.right.frame_id", "cam1");
        config.right.encoding = declare_parameter<std::string>(
            "image.right.encoding", "mono8");
        return config;
        }

        ImuConfig loadImuConfig() {
        ImuConfig config{};
        config.topic = declare_parameter<std::string>("imu.topic", "/imu0");
        config.frame_id = declare_parameter<std::string>("imu.frame_id", "imu0");
        config.gravity_magnitude =
            declare_parameter<double>("imu.gravity_magnitude", 9.7887);
        config.publish_hz = declare_parameter<double>("imu.publish_hz", 200.0);
        return config;
        }

        DebugConfig loadDebugConfig() {
        DebugConfig config{};
        config.log_image_timestamps =
            declare_parameter<bool>("debug.log_image_timestamps", false);
        config.log_imu_timestamps =
            declare_parameter<bool>("debug.log_imu_timestamps", false);
        return config;
        }

    rclcpp::QoS loadQosConfig(const std::string &prefix, int default_depth) {
        TopicQosConfig config{};
        config.history = declare_parameter<std::string>(prefix + ".history", "keep_last");
        config.depth = declare_parameter<int>(prefix + ".depth", default_depth);
        config.reliability =
                declare_parameter<std::string>(prefix + ".reliability", "reliable");
        config.durability =
                declare_parameter<std::string>(prefix + ".durability", "volatile");

        rclcpp::QoS qos(rclcpp::KeepLast(std::max(1, config.depth)));

        if (toLowerCopy(config.history) == "keep_all") {
            qos.keep_all();
        } else {
            qos.keep_last(std::max(1, config.depth));
        }

        if (toLowerCopy(config.reliability) == "best_effort") {
            qos.best_effort();
        } else {
            qos.reliable();
        }

        if (toLowerCopy(config.durability) == "transient_local") {
            qos.transient_local();
        } else {
            qos.durability_volatile();
        }

        return qos;
    }

    void enqueueBatch(CaptureBatch batch) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (capture_queue_.size() >= static_cast<size_t>(std::max(1, capture_config_.queue_size))) {
            if (shouldDropOldest(capture_config_.queue_drop_policy)) {
                capture_queue_.pop_front();
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                                         "Capture queue full, dropping oldest batch.");
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                                         "Capture queue full, dropping newest batch.");
                return;
            }
        }
        capture_queue_.push_back(std::move(batch));
        queue_cv_.notify_one();
    }

    bool dequeueBatch(CaptureBatch &batch) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
            return !is_running_ || !capture_queue_.empty();
        });

        if (capture_queue_.empty()) {
            return false;
        }

        batch = std::move(capture_queue_.front());
        capture_queue_.pop_front();
        return true;
    }

    void captureLoop() {
        while (is_running_) {
            std::shared_ptr<cyperstereo::uvc::device> cyperstereo_device{nullptr};
            if (!cyperstereo::FindCyperstereoDevices(cyperstereo_device)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                                         "No Cyperstereo device, retrying...");
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(capture_config_.reconnect_delay_ms));
                continue;
            }

            cyperstereo::FrameInfo frame_info{};
            frame_info.ResetState();

            cyperstereo::uvc::set_device_mode(
                    *cyperstereo_device, device_config_.width, device_config_.height,
                    static_cast<int>(resolvePixelFormat(device_config_.pixel_format)),
                    device_config_.fps,
                    [&frame_info](const void *data, std::function<void()> continuation) {
                        cyperstereo::SetStreamData(frame_info, data, continuation);
                    });

            try {
                cyperstereo::uvc::start_streaming(*cyperstereo_device,
                                                                                    device_config_.stream_index);
                int invalid_timestamp_batch_count = 0;
                while (is_running_) {
                    cyperstereo::WaitForStream(frame_info);

                    CaptureBatch batch;
                    batch.image_timestamp = frame_info.framestream.image_timestamp;
                    batch.left_image = frame_info.framestream.left_image.clone();
                    batch.right_image = frame_info.framestream.right_image.clone();

                    const int imu_sample_count =
                            std::max(0, frame_info.framestream.imu.imu_count + 1);
                    batch.imu_samples.reserve(static_cast<size_t>(imu_sample_count));
                    for (int index = 0; index < imu_sample_count; ++index) {
                        ImuSample sample;
                        sample.timestamp = frame_info.framestream.imu.imu_timestamp[index];
                        sample.gyro_x = frame_info.framestream.imu.gyro_x[index];
                        sample.gyro_y = frame_info.framestream.imu.gyro_y[index];
                        sample.gyro_z = frame_info.framestream.imu.gyro_z[index];
                        sample.acc_x = frame_info.framestream.imu.acc_x[index] *
                                                     imu_config_.gravity_magnitude;
                        sample.acc_y = frame_info.framestream.imu.acc_y[index] *
                                                     imu_config_.gravity_magnitude;
                        sample.acc_z = frame_info.framestream.imu.acc_z[index] *
                                                     imu_config_.gravity_magnitude;
                        if (isValidDeviceTimestamp(sample.timestamp)) {
                            batch.imu_samples.push_back(sample);
                        }
                    }

                    const bool image_timestamp_valid =
                        isValidDeviceTimestamp(batch.image_timestamp);
                    const bool has_valid_imu = !batch.imu_samples.empty();
                    if (!image_timestamp_valid || !has_valid_imu) {
                        ++invalid_timestamp_batch_count;
                        const int warn_period = std::max(1, device_config_.fps);
                        if (invalid_timestamp_batch_count == 1 ||
                            invalid_timestamp_batch_count % warn_period == 0) {
                            RCLCPP_WARN(
                                get_logger(),
                                "Skipping invalid capture batch: image_timestamp=%.6f, valid_imu=%zu, consecutive_invalid=%d",
                                batch.image_timestamp, batch.imu_samples.size(),
                                invalid_timestamp_batch_count);
                        }

                        const int restart_threshold = std::max(10, device_config_.fps * 2);
                        if (invalid_timestamp_batch_count >= restart_threshold) {
                            throw std::runtime_error(
                                "Device kept producing invalid timestamps after reconnect.");
                        }
                        continue;
                    }

                    if (invalid_timestamp_batch_count > 0) {
                        RCLCPP_INFO(get_logger(),
                                                "Recovered valid timestamps after %d invalid batches.",
                                                invalid_timestamp_batch_count);
                        invalid_timestamp_batch_count = 0;
                    }

                    if (debug_config_.log_image_timestamps) {
                        RCLCPP_INFO(get_logger(), "image_timestamp %.6f, imu_count=%zu",
                                                batch.image_timestamp, batch.imu_samples.size());
                    }

                    if (debug_config_.log_imu_timestamps) {
                        for (const auto &sample : batch.imu_samples) {
                            RCLCPP_INFO(get_logger(),
                                                    "imu_timestamp %.6f gyro=[%.6f %.6f %.6f] acc=[%.6f %.6f %.6f]",
                                                    sample.timestamp, sample.gyro_x, sample.gyro_y,
                                                    sample.gyro_z, sample.acc_x, sample.acc_y,
                                                    sample.acc_z);
                        }
                    }

                    enqueueBatch(std::move(batch));
                }

                cyperstereo::uvc::stop_streaming(*cyperstereo_device);
            } catch (const std::exception &e) {
                RCLCPP_WARN(get_logger(),
                                        "capture loop error: %s, restarting when device is back.",
                                        e.what());
                try {
                    cyperstereo::uvc::stop_streaming(*cyperstereo_device);
                } catch (const std::exception &stop_error) {
                    RCLCPP_WARN(get_logger(), "stop_streaming error: %s", stop_error.what());
                }
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(capture_config_.reconnect_delay_ms));
            }
        }
    }

    void publishLoop() {
        while (is_running_) {
            CaptureBatch batch;
            if (!dequeueBatch(batch)) {
                if (!is_running_) {
                    break;
                }
                continue;
            }

            publishBatch(batch);
        }
    }

    void publishBatch(const CaptureBatch &batch) {
        if (!isValidDeviceTimestamp(batch.image_timestamp)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                                     "Skipping batch with invalid image timestamp.");
            return;
        }

        const bool publish_image = shouldPublishTimestamp(
            batch.image_timestamp, image_config_.publish_hz,
            last_published_image_timestamp_);

        const rclcpp::Time image_stamp = toRosTime(batch.image_timestamp);

        for (const auto &sample : batch.imu_samples) {
            if (!isValidDeviceTimestamp(sample.timestamp)) {
                continue;
            }

            if (!shouldPublishTimestamp(sample.timestamp, imu_config_.publish_hz,
                                        last_published_imu_timestamp_)) {
                continue;
            }

            sensor_msgs::msg::Imu imu_data;
            imu_data.header.stamp = toRosTime(sample.timestamp);
            imu_data.header.frame_id = imu_config_.frame_id;
            imu_data.linear_acceleration.x = sample.acc_x;
            imu_data.linear_acceleration.y = sample.acc_y;
            imu_data.linear_acceleration.z = sample.acc_z;
            imu_data.angular_velocity.x = sample.gyro_x;
            imu_data.angular_velocity.y = sample.gyro_y;
            imu_data.angular_velocity.z = sample.gyro_z;
            imu_pub_->publish(imu_data);
        }

        if (!publish_image) {
            return;
        }

        auto left_msg = cv_bridge::CvImage(
                                                std_msgs::msg::Header(), image_config_.left.encoding, batch.left_image)
                                                .toImageMsg();
        left_msg->header.stamp = image_stamp;
        left_msg->header.frame_id = image_config_.left.frame_id;

        auto right_msg = cv_bridge::CvImage(
                                                 std_msgs::msg::Header(), image_config_.right.encoding, batch.right_image)
                                                 .toImageMsg();
        right_msg->header.stamp = image_stamp;
        right_msg->header.frame_id = image_config_.right.frame_id;

        cam0_image_pub_->publish(*left_msg);
        cam1_image_pub_->publish(*right_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam0_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam1_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::thread producer_thread_;
    std::thread consumer_thread_;
    std::atomic<bool> is_running_;

    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<CaptureBatch> capture_queue_;

    DeviceConfig device_config_;
    CaptureConfig capture_config_;
    ImageConfig image_config_;
    ImuConfig imu_config_;
    DebugConfig debug_config_;
    double last_published_image_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    double last_published_imu_timestamp_ = std::numeric_limits<double>::quiet_NaN();
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CyperstereoRos2BridgeNode>());
    rclcpp::shutdown();
    return 0;
}
